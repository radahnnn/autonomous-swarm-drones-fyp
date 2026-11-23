/*
 * ESP32 TWAI (CAN) bring-up diagnostic - READ-ONLY by default (listen-only), prints bus health every second.
 *
 * Purpose: find out whether the CAN bus between this ESP32 (via an MCP2551 breakout "MCU-2551") and the Matek H743
 * flight controller is electrically alive. The Matek runs DroneCAN at 250 kbit/s (CAN_P1_DRIVER=1, CAN_P1_BITRATE=250000,
 * node 10) and should broadcast uavcan.protocol.NodeStatus (data type id 341) about once per second.
 *
 * WIRING (set the pin numbers below to match what you actually wire; defaults are GPIO5 / GPIO4):
 *   ESP32 GPIO5 (CAN_TX_PIN)  -> MCU-2551 CTX
 *   ESP32 GPIO4 (CAN_RX_PIN)  <- MCU-2551 CRX   *** through a voltage divider, see below ***
 *   ESP32 5V pin (USB 5 V)    -> MCU-2551 VCC   (MCP2551 is a 5 V part)
 *   ESP32 GND                 -> MCU-2551 GND  and to the Matek GND (common reference)
 *   MCU-2551 CANH -> Matek CAN-H,  MCU-2551 CANL -> Matek CAN-L
 * CRX is a 5 V push-pull output when the module is powered from 5 V. Do NOT connect it straight to an ESP32 GPIO (3.3 V max):
 * use a divider, e.g. CRX --1k--+--> GPIO4, and GPIO4 --2k--> GND (about 3.3 V at the pin), or use a 3.3 V transceiver (SN65HVD230).
 * Termination: a CAN bus needs 120 ohm at each end. Many MCP2551 breakouts have a 120 ohm jumper/pads; check yours.
 *
 * SAFETY: this sketch never sends a frame in listen-only mode and never touches the flight controller's parameters.
 * Flash it only with the flight battery disconnected. Flashing REPLACES the current ESP32 firmware (it prints
 * "Waiting for CAN packets..."); keep a copy of the old source/binary first if you want it back.
 *
 * Build: Arduino IDE, board "ESP32 Dev Module", Serial Monitor 115200 baud. Uses the ESP-IDF TWAI driver bundled with arduino-esp32.
 */
#include "driver/twai.h"

#define CAN_TX_PIN   GPIO_NUM_5
#define CAN_RX_PIN   GPIO_NUM_4
#define LISTEN_ONLY  1        // 1 = never transmit or ACK (pure observation); 0 = normal mode (ACKs frames, still sends nothing else)
#define MAX_FRAME_PRINTS_PER_SECOND 10

static const char *stateName(twai_state_t s) {
  switch (s) {
    case TWAI_STATE_STOPPED:    return "STOPPED";
    case TWAI_STATE_RUNNING:    return "RUNNING";
    case TWAI_STATE_BUS_OFF:    return "BUS_OFF";
    case TWAI_STATE_RECOVERING: return "RECOVERING";
  }
  return "?";
}

static uint32_t framesTotal = 0, nodeStatusTotal = 0;
static unsigned long lastStatusMs = 0;
static int printsThisSecond = 0;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.print("TWAI diagnostic: 250 kbit/s, ");
  Serial.println(LISTEN_ONLY ? "LISTEN-ONLY" : "NORMAL");
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, LISTEN_ONLY ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL);
  g.rx_queue_len = 64;
  g.alerts_enabled = TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) { Serial.println("ERROR: twai_driver_install failed"); return; }
  if (twai_start() != ESP_OK)                    { Serial.println("ERROR: twai_start failed"); return; }
  Serial.println("TWAI started. Waiting for frames; status every second.");
}

static void describeFrame(const twai_message_t &m) {
  Serial.printf("  FRAME id=0x%08lX %s dlc=%d data=", (unsigned long)m.identifier, m.extd ? "ext" : "std", m.data_length_code);
  for (int i = 0; i < m.data_length_code; i++) Serial.printf("%02X ", m.data[i]);
  // DroneCAN (UAVCAN v0) message frames use 29-bit ids: [priority:5][data type id:16][service-not-message:1][source node id:7]
  if (m.extd && !(m.identifier & 0x80)) {
    uint16_t dtid = (m.identifier >> 8) & 0xFFFF;
    uint8_t node = m.identifier & 0x7F;
    Serial.printf("| DroneCAN msg type %u from node %u", dtid, node);
    if (dtid == 341 && m.data_length_code == 8) {   // NodeStatus: 7 payload bytes + tail byte
      uint32_t uptime = (uint32_t)m.data[0] | ((uint32_t)m.data[1] << 8) | ((uint32_t)m.data[2] << 16) | ((uint32_t)m.data[3] << 24);
      Serial.printf(" = NodeStatus uptime=%lus health=%u mode=%u", (unsigned long)uptime, (m.data[4] >> 6) & 3, (m.data[4] >> 3) & 7);
      nodeStatusTotal++;
    }
  }
  Serial.println();
}

void loop() {
  twai_message_t m;
  while (twai_receive(&m, 0) == ESP_OK) {
    framesTotal++;
    if (printsThisSecond < MAX_FRAME_PRINTS_PER_SECOND) { describeFrame(m); printsThisSecond++; }
  }
  uint32_t alerts;
  if (twai_read_alerts(&alerts, 0) == ESP_OK) {
    if (alerts & TWAI_ALERT_BUS_OFF)       Serial.println("ALERT: bus-off");
    if (alerts & TWAI_ALERT_ERR_PASS)      Serial.println("ALERT: error-passive");
    if (alerts & TWAI_ALERT_RX_QUEUE_FULL) Serial.println("ALERT: rx queue full");
  }
  if (millis() - lastStatusMs >= 1000) {
    lastStatusMs = millis();
    printsThisSecond = 0;
    twai_status_info_t s;
    if (twai_get_status_info(&s) == ESP_OK) {
      Serial.printf("[status] state=%s frames_total=%lu nodestatus_total=%lu msgs_in_queue=%lu rx_err=%lu tx_err=%lu bus_err=%lu rx_missed=%lu arb_lost=%lu\n",
                    stateName(s.state), (unsigned long)framesTotal, (unsigned long)nodeStatusTotal, (unsigned long)s.msgs_to_rx,
                    (unsigned long)s.rx_error_counter, (unsigned long)s.tx_error_counter, (unsigned long)s.bus_error_count,
                    (unsigned long)s.rx_missed_count, (unsigned long)s.arb_lost_count);
    }
  }
}
