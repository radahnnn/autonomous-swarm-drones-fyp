/*
 * ESP32 TWAI (CAN) bring-up diagnostic - READ-ONLY by default (listen-only), prints bus health every second.
 *
 * Purpose: find out whether the CAN bus between this ESP32 (via an MCP2551 breakout "MCU-2551") and the Matek H743
 * flight controller is electrically alive in BOTH directions (RX first, then TX). The Matek runs DroneCAN at
 * 250 kbit/s (CAN_P1_DRIVER=1, CAN_P1_BITRATE=250000, node 10) and broadcasts uavcan.protocol.NodeStatus (dtid 341)
 * about once per second.
 *
 * CRITICAL MATEK H743-SLIM V3 POWER FACT:
 *   The Matek CAN1 5 V rail and onboard CAN transceiver are powered from the onboard 5 V BEC (LiPo VBAT), NOT from
 *   USB VBUS. On USB-only power the Matek MCU runs, but CAN1 is unpowered and silent.
 *   Therefore: REMOVE ALL 4 PROPELLERS PHYSICALLY before connecting the flight battery (VBAT) for this test.
 *
 * WIRING (set the pin numbers below to match what you actually wired; defaults are GPIO5 / GPIO4):
 *   ESP32 GPIO5 (CAN_TX_PIN)  -> MCU-2551 CTX
 *   ESP32 GPIO4 (CAN_RX_PIN)  <- MCU-2551 CRX   *** through a voltage divider (e.g. CRX --1k--+--> GPIO4 --2k--> GND) ***
 *   ESP32 5V pin (USB 5 V)    -> MCU-2551 VCC   (MCP2551 is a 5 V part)
 *   ESP32 GND                 -> MCU-2551 GND  and to the Matek CAN1 G (common reference)
 *   MCU-2551 CANH -> Matek CAN-H,  MCU-2551 CANL -> Matek CAN-L
 *
 * TX NOTE (ESP32 3.3 V -> MCU-2551 CTX):
 *   Test in two steps:
 *     1) LISTEN_ONLY = 1: verify Matek NodeStatus frames arrive cleanly (rx_err=0, bus_err=0).
 *     2) LISTEN_ONLY = 0 (SEND_TEST_NODESTATUS = 1): verifies ESP32 TX -> MCU-2551 CTX works (tx_ok climbs, tx_err=0).
 *   If Step 1 passes cleanly but Step 2 shows tx_err climbing, MCU-2551 is rejecting 3.3 V HIGH on CTX (spec 0.7*VDD):
 *   add a breadboard-only buffer (74HCT08 or 74HCT125 at 5 V, or a BSS138 3.3V<->5V module, or a 1N4007 diode in
 *   series with MCU-2551 VCC) between ESP32 TX and MCU-2551 CTX.
 *
 * Termination: a CAN bus needs 120 ohm at each end. Check whether the 120 ohm jumper/pads on MCU-2551 are closed.
 *
 * SAFETY:
 *   - Listen-only mode never transmits or ACKs. Even with LISTEN_ONLY = 0, this sketch sends ONLY NodeStatus
 *     (never MAVLink, never arm, never motor commands) and never touches FC parameters.
 *   - Flashing REPLACES the current ESP32 firmware ("Waiting for CAN packets..."); back up old flash with esptool
 *     first if you want a copy. Keep the flight battery disconnected while flashing.
 *
 * Build: Arduino IDE, board "ESP32 Dev Module", Serial Monitor 115200 baud. Uses ESP-IDF TWAI bundled with arduino-esp32.
 * Status: NOT YET COMPILED OR TESTED ON PHYSICAL HARDWARE.
 */
#include "driver/twai.h"

#define CAN_TX_PIN            GPIO_NUM_5
#define CAN_RX_PIN            GPIO_NUM_4
#define LISTEN_ONLY           1   // 1 = RX-only observation; 0 = NORMAL mode (ACKs + sends NodeStatus if enabled below)
#define SEND_TEST_NODESTATUS  1   // Used ONLY when LISTEN_ONLY == 0: broadcasts 1 Hz NodeStatus from ESP32_CAN_NODE_ID
#define ESP32_CAN_NODE_ID     50  // 1..127, must differ from Matek CAN_D1_UC_NODE (10)
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
static uint32_t txOkTotal = 0, txFailTotal = 0;
static uint8_t txTid = 0;
static unsigned long lastStatusMs = 0;
static int printsThisSecond = 0;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.print("TWAI diagnostic: 250 kbit/s, ");
  Serial.println(LISTEN_ONLY ? "LISTEN-ONLY (RX only)" : "NORMAL (ACK + NodeStatus TX)");
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, LISTEN_ONLY ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL);
  g.rx_queue_len = 64;
  g.alerts_enabled = TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) { Serial.println("ERROR: twai_driver_install failed"); return; }
  if (twai_start() != ESP_OK)                    { Serial.println("ERROR: twai_start failed"); return; }
  Serial.println("TWAI started. Waiting for frames; status every second.");
}

static void sendTestNodeStatus(uint32_t uptimeSec) {
  twai_message_t tx = {};
  tx.identifier = (16UL << 24) | ((uint32_t)341 << 8) | (ESP32_CAN_NODE_ID & 0x7F);
  tx.extd = 1;
  tx.data_length_code = 8;
  tx.data[0] = (uint8_t)(uptimeSec & 0xFF);
  tx.data[1] = (uint8_t)((uptimeSec >> 8) & 0xFF);
  tx.data[2] = (uint8_t)((uptimeSec >> 16) & 0xFF);
  tx.data[3] = (uint8_t)((uptimeSec >> 24) & 0xFF);
  tx.data[4] = 0x00;  // health=OK(0), mode=OPERATIONAL(0), sub_mode=0
  tx.data[5] = 0x00;
  tx.data[6] = 0x00;
  tx.data[7] = (uint8_t)(0xC0 | (txTid & 0x1F));  // SOT=1, EOT=1, toggle=0
  txTid = (uint8_t)((txTid + 1) & 0x1F);
  if (twai_transmit(&tx, pdMS_TO_TICKS(10)) == ESP_OK) {
    txOkTotal++;
  } else {
    txFailTotal++;
  }
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
    if (!LISTEN_ONLY && SEND_TEST_NODESTATUS) {
      sendTestNodeStatus((uint32_t)(lastStatusMs / 1000UL));
    }
    twai_status_info_t s;
    if (twai_get_status_info(&s) == ESP_OK) {
      Serial.printf("[status] state=%s frames_total=%lu nodestatus_total=%lu tx_ok=%lu tx_fail=%lu msgs_in_queue=%lu rx_err=%lu tx_err=%lu bus_err=%lu rx_missed=%lu arb_lost=%lu\n",
                    stateName(s.state), (unsigned long)framesTotal, (unsigned long)nodeStatusTotal,
                    (unsigned long)txOkTotal, (unsigned long)txFailTotal, (unsigned long)s.msgs_to_rx,
                    (unsigned long)s.rx_error_counter, (unsigned long)s.tx_error_counter, (unsigned long)s.bus_error_count,
                    (unsigned long)s.rx_missed_count, (unsigned long)s.arb_lost_count);
    }
  }
}
