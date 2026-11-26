/*
 * ESP32 DroneCAN Serial-Tunnel <-> Wi-Fi UDP MAVLink2 Bridge
 *
 * Bridges MAVLink2 bytes between ArduPilot (Matek H743-SLIM V3 CAN1 via uavcan.tunnel.Targetted)
 * and a ground station / IndependentAgentNode over Wi-Fi UDP, with zero soldering on the Matek.
 *
 * Matek H743-SLIM V3 parameters required (set via USB in QGroundControl):
 *   CAN_P1_DRIVER    = 1       (DroneCAN)
 *   CAN_P1_BITRATE   = 250000  (match CAN_BITRATE_KBPS below; 250/500/1000)
 *   CAN_D1_PROTOCOL  = 1
 *   CAN_D1_UC_NODE   = 10
 *   CAN_D1_UC_SER_EN = 1       (reboot Matek after setting this so S1 params appear)
 *   CAN_D1_UC_S1_NOD = 50      (match ESP32_CAN_NODE_ID below)
 *   CAN_D1_UC_S1_IDX = 0       (match SERIAL_ID below)
 *   CAN_D1_UC_S1_PRO = 2       (MAVLink2)
 *   CAN_D1_UC_S1_BD  = 115200  (match TUNNEL_BAUDRATE below)
 *
 * Drone 1 vs Drone 2 configuration:
 *   Drone 1: Matek SYSID_THISMAV=1, GCS_UDP_PORT=14551
 *   Drone 2: Matek SYSID_THISMAV=2, GCS_UDP_PORT=14552
 *
 * SAFETY:
 *   - Remove all 4 propellers physically before connecting LiPo VBAT on the bench.
 *   - Back up existing ESP32 flash before flashing if you want to preserve prior firmware.
 *
 * Build: Arduino IDE, board "ESP32 Dev Module" (no external libraries required), Serial Monitor 115200.
 * Status: Codec UNIT-tested on host against ArduPilot libcanard; .ino sketch NOT YET COMPILED OR TESTED ON HARDWARE.
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include "driver/twai.h"
#include "dronecan_tunnel_codec.h"

/* ---------------- USER CONFIGURATION ---------------- */
#define WIFI_SSID          "YOUR_WIFI_SSID"
#define WIFI_PASS          "YOUR_WIFI_PASSWORD"
#define GCS_IP             "192.168.1.100"   // Laptop IP running QGC / IndependentAgentNode
#define GCS_UDP_PORT       14551             // 14551 for Drone 1, 14552 for Drone 2
#define LOCAL_UDP_PORT     14550             // Local UDP port listened on by this ESP32

#define CAN_TX_PIN         GPIO_NUM_5        // ESP32 TX -> MCU-2551 CTX
#define CAN_RX_PIN         GPIO_NUM_4        // ESP32 RX <- MCU-2551 CRX (via voltage divider!)
#define CAN_BITRATE_KBPS   250               // 250, 500, or 1000 (must match Matek CAN_P1_BITRATE)

#define ESP32_CAN_NODE_ID  50                // Must match CAN_D1_UC_S1_NOD
#define MATEK_CAN_NODE_ID  10                // Must match CAN_D1_UC_NODE
#define SERIAL_ID          0                 // Must match CAN_D1_UC_S1_IDX
#define TUNNEL_BAUDRATE    115200UL          // Informational baudrate field in Targetted
/* ---------------------------------------------------- */

static WiFiUDP udp;
static IPAddress peerIp;
static uint16_t peerPort = GCS_UDP_PORT;

static DcTunnelRx tunnelRx;
static uint8_t nodeStatusTid = 0;
static uint8_t tunnelTxTid = 0;

static uint32_t canRxBytes = 0;
static uint32_t canTxBytes = 0;
static uint32_t udpRxBytes = 0;
static uint32_t udpTxBytes = 0;
static uint32_t canTxDrops = 0;

static unsigned long lastNodeStatusMs = 0;
static unsigned long lastDiagMs = 0;
static unsigned long lastWifiRetryMs = 0;

static const char *twaiStateName(twai_state_t s) {
  switch (s) {
    case TWAI_STATE_STOPPED:    return "STOPPED";
    case TWAI_STATE_RUNNING:    return "RUNNING";
    case TWAI_STATE_BUS_OFF:    return "BUS_OFF";
    case TWAI_STATE_RECOVERING: return "RECOVERING";
  }
  return "?";
}

static bool initTwai() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  g.rx_queue_len = 64;
  g.tx_queue_len = 32;
  g.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_ERR_PASS | TWAI_ALERT_RX_QUEUE_FULL;

#if CAN_BITRATE_KBPS == 250
  twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS();
#elif CAN_BITRATE_KBPS == 500
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
#elif CAN_BITRATE_KBPS == 1000
  twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
#else
#error "Unsupported CAN_BITRATE_KBPS (use 250, 500, or 1000)"
#endif

  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) {
    Serial.println("ERROR: twai_driver_install failed");
    return false;
  }
  if (twai_start() != ESP_OK) {
    Serial.println("ERROR: twai_start failed");
    return false;
  }
  return true;
}

static bool sendCanFrame(const DcCanFrame &f) {
  twai_message_t tx = {};
  tx.identifier = f.id;
  tx.extd = 1;
  tx.data_length_code = f.dlc;
  memcpy(tx.data, f.data, f.dlc);
  return twai_transmit(&tx, pdMS_TO_TICKS(10)) == ESP_OK;
}

static void sendTunnelChunk(const uint8_t *data, uint8_t len) {
  uint8_t payload[DC_TUNNEL_MAX_PAYLOAD];
  uint8_t payloadLen = dc_encode_tunnel_payload(
      DC_TUNNEL_PROTO_MAVLINK2,
      MATEK_CAN_NODE_ID,
      SERIAL_ID,
      DC_TUNNEL_OPT_LOCK_PORT,
      TUNNEL_BAUDRATE,
      data,
      len,
      payload);

  DcCanFrame frames[DC_TUNNEL_MAX_FRAMES];
  size_t nFrames = dc_frame_transfer(
      16U,
      DC_TUNNEL_TARGETTED_ID,
      DC_TUNNEL_TARGETTED_SIGNATURE,
      ESP32_CAN_NODE_ID,
      payload,
      payloadLen,
      &tunnelTxTid,
      frames,
      DC_TUNNEL_MAX_FRAMES);

  for (size_t i = 0; i < nFrames; i++) {
    if (!sendCanFrame(frames[i])) {
      canTxDrops++;
      return;
    }
  }
  canTxBytes += len;
}

static void maintainWifi(unsigned long nowMs) {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }
  if (nowMs - lastWifiRetryMs >= 5000UL) {
    lastWifiRetryMs = nowMs;
    Serial.println("Wi-Fi disconnected; reconnecting...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.printf("DroneCAN<->UDP MAVLink2 bridge starting (CAN=%d kbps, node=%d -> matek=%d)\n",
                CAN_BITRATE_KBPS, ESP32_CAN_NODE_ID, MATEK_CAN_NODE_ID);

  peerIp.fromString(GCS_IP);
  dc_tunnel_rx_init(&tunnelRx, MATEK_CAN_NODE_ID, ESP32_CAN_NODE_ID, SERIAL_ID);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  udp.begin(LOCAL_UDP_PORT);

  if (!initTwai()) {
    Serial.println("TWAI init failed — check CAN pins and reboot.");
  } else {
    Serial.println("TWAI started.");
  }
}

void loop() {
  unsigned long nowMs = millis();
  maintainWifi(nowMs);

  /* 1. CAN -> Wi-Fi UDP (drain incoming DroneCAN tunnel frames and batch into UDP packet) */
  uint8_t outUdpBuf[512];
  size_t outUdpLen = 0;
  twai_message_t rxMsg;
  while (twai_receive(&rxMsg, 0) == ESP_OK) {
    if (!rxMsg.extd || rxMsg.rtr) {
      continue;
    }
    DcCanFrame f;
    f.id = rxMsg.identifier;
    f.dlc = rxMsg.data_length_code;
    memcpy(f.data, rxMsg.data, f.dlc <= 8 ? f.dlc : 8);

    uint8_t chunk[DC_TUNNEL_MAX_DATA];
    uint8_t chunkLen = 0;
    if (dc_tunnel_rx_feed(&tunnelRx, &f, (uint32_t)nowMs, chunk, &chunkLen)) {
      if (chunkLen > 0) {
        canRxBytes += chunkLen;
        if (outUdpLen + chunkLen > sizeof(outUdpBuf)) {
          if (WiFi.status() == WL_CONNECTED && outUdpLen > 0) {
            udp.beginPacket(peerIp, peerPort);
            udp.write(outUdpBuf, outUdpLen);
            udp.endPacket();
            udpTxBytes += outUdpLen;
          }
          outUdpLen = 0;
        }
        memcpy(&outUdpBuf[outUdpLen], chunk, chunkLen);
        outUdpLen += chunkLen;
      }
    }
  }
  if (outUdpLen > 0 && WiFi.status() == WL_CONNECTED) {
    udp.beginPacket(peerIp, peerPort);
    udp.write(outUdpBuf, outUdpLen);
    udp.endPacket();
    udpTxBytes += outUdpLen;
  }

  /* 2. Wi-Fi UDP -> CAN (forward incoming GCS/agent MAVLink2 packets over DroneCAN tunnel) */
  if (WiFi.status() == WL_CONNECTED) {
    int pktSize = udp.parsePacket();
    while (pktSize > 0) {
      peerIp = udp.remoteIP();
      peerPort = udp.remotePort();
      uint8_t inBuf[512];
      int toRead = pktSize > (int)sizeof(inBuf) ? (int)sizeof(inBuf) : pktSize;
      int n = udp.read(inBuf, toRead);
      if (n > 0) {
        udpRxBytes += (uint32_t)n;
        int off = 0;
        while (off < n) {
          uint8_t seg = (uint8_t)((n - off) > (int)DC_TUNNEL_MAX_DATA ? DC_TUNNEL_MAX_DATA : (n - off));
          sendTunnelChunk(&inBuf[off], seg);
          off += seg;
        }
      }
      pktSize = udp.parsePacket();
    }
  }

  /* 3. Broadcast 1 Hz DroneCAN NodeStatus & handle TWAI alerts */
  if (nowMs - lastNodeStatusMs >= 1000UL) {
    lastNodeStatusMs = nowMs;
    DcCanFrame nsFrame;
    dc_encode_nodestatus_frame(
        ESP32_CAN_NODE_ID,
        (uint32_t)(nowMs / 1000UL),
        DC_NODESTATUS_HEALTH_OK,
        DC_NODESTATUS_MODE_OPERATIONAL,
        &nodeStatusTid,
        &nsFrame);
    sendCanFrame(nsFrame);
  }

  uint32_t alerts = 0;
  if (twai_read_alerts(&alerts, 0) == ESP_OK) {
    if (alerts & TWAI_ALERT_BUS_OFF) {
      Serial.println("ALERT: TWAI BUS_OFF — initiating recovery");
      twai_initiate_recovery();
    }
  }

  /* 4. 1 Hz diagnostics */
  if (nowMs - lastDiagMs >= 1000UL) {
    lastDiagMs = nowMs;
    twai_status_info_t st;
    const char *stName = (twai_get_status_info(&st) == ESP_OK) ? twaiStateName(st.state) : "?";
    uint32_t rxErr = (twai_get_status_info(&st) == ESP_OK) ? st.rx_error_counter : 0;
    uint32_t txErr = (twai_get_status_info(&st) == ESP_OK) ? st.tx_error_counter : 0;
    uint32_t busErr = (twai_get_status_info(&st) == ESP_OK) ? st.bus_error_count : 0;
    Serial.printf("[bridge] wifi=%s ip=%s peer=%s:%u can=%s rx_b=%lu tx_b=%lu udp_rx=%lu udp_tx=%lu crc_err=%lu frm_err=%lu drop=%lu rx_err=%lu tx_err=%lu bus_err=%lu\n",
                  WiFi.status() == WL_CONNECTED ? "OK" : "DOWN",
                  WiFi.localIP().toString().c_str(),
                  peerIp.toString().c_str(),
                  (unsigned)peerPort,
                  stName,
                  (unsigned long)canRxBytes,
                  (unsigned long)canTxBytes,
                  (unsigned long)udpRxBytes,
                  (unsigned long)udpTxBytes,
                  (unsigned long)tunnelRx.rx_crc_errors,
                  (unsigned long)tunnelRx.rx_frame_errors,
                  (unsigned long)canTxDrops,
                  (unsigned long)rxErr,
                  (unsigned long)txErr,
                  (unsigned long)busErr);
  }
}
