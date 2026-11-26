#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* DroneCAN (UAVCAN v0) constants verified against ArduPilot 4.6 dsdlc_generated */
#define DC_NODESTATUS_ID              341U
#define DC_NODESTATUS_SIGNATURE       0x0F0868D0C1A7C6F1ULL
#define DC_NODESTATUS_HEALTH_OK       0U
#define DC_NODESTATUS_MODE_OPERATIONAL 0U

#define DC_TUNNEL_TARGETTED_ID        3001U
#define DC_TUNNEL_TARGETTED_SIGNATURE 0xB138E7EA72A2A2E9ULL
#define DC_TUNNEL_PROTO_MAVLINK       0U
#define DC_TUNNEL_PROTO_MAVLINK2      1U
#define DC_TUNNEL_OPT_LOCK_PORT       1U

#define DC_TUNNEL_HEADER_LEN          6U
#define DC_TUNNEL_MAX_DATA            120U
#define DC_TUNNEL_MAX_PAYLOAD         (DC_TUNNEL_HEADER_LEN + DC_TUNNEL_MAX_DATA)
#define DC_TUNNEL_MAX_FRAMES          20U
#define DC_TUNNEL_RX_TIMEOUT_MS       500U

typedef struct {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
} DcCanFrame;

typedef struct {
  uint8_t expected_src_node;
  uint8_t my_node_id;
  int8_t expected_serial_id;
  bool in_progress;
  uint8_t tid;
  uint8_t expected_toggle;
  uint16_t expected_crc;
  uint32_t start_ms;
  uint16_t len;
  uint8_t buf[DC_TUNNEL_MAX_PAYLOAD];
  uint32_t rx_transfers_ok;
  uint32_t rx_crc_errors;
  uint32_t rx_frame_errors;
  uint32_t rx_bytes;
} DcTunnelRx;

static inline uint32_t dc_make_broadcast_id(uint8_t priority, uint16_t dtid, uint8_t src_node) {
  return (((uint32_t)(priority & 0x1FU)) << 24U) |
         (((uint32_t)dtid) << 8U) |
         ((uint32_t)(src_node & 0x7FU));
}

static inline uint16_t dc_crc16_byte(uint16_t crc, uint8_t b) {
  crc ^= (uint16_t)((uint16_t)b << 8U);
  for (uint8_t j = 0; j < 8U; j++) {
    if (crc & 0x8000U) {
      crc = (uint16_t)((uint16_t)(crc << 1U) ^ 0x1021U);
    } else {
      crc = (uint16_t)(crc << 1U);
    }
  }
  return crc;
}

static inline uint16_t dc_crc16_update(uint16_t crc, const uint8_t *buf, size_t len) {
  for (size_t i = 0; i < len; i++) {
    crc = dc_crc16_byte(crc, buf[i]);
  }
  return crc;
}

static inline uint16_t dc_crc16_init(uint64_t data_type_signature) {
  uint16_t crc = 0xFFFFU;
  for (uint8_t shift = 0; shift < 64U; shift = (uint8_t)(shift + 8U)) {
    crc = dc_crc16_byte(crc, (uint8_t)(data_type_signature >> shift));
  }
  return crc;
}

static inline void dc_encode_nodestatus_frame(
    uint8_t src_node,
    uint32_t uptime_sec,
    uint8_t health,
    uint8_t mode,
    uint8_t *inout_tid,
    DcCanFrame *out_frame) {
  uint8_t tid = (uint8_t)((*inout_tid) & 0x1FU);
  *inout_tid = (uint8_t)((tid + 1U) & 0x1FU);

  memset(out_frame, 0, sizeof(*out_frame));
  out_frame->id = dc_make_broadcast_id(16U, DC_NODESTATUS_ID, src_node);
  out_frame->dlc = 8U;
  out_frame->data[0] = (uint8_t)(uptime_sec & 0xFFU);
  out_frame->data[1] = (uint8_t)((uptime_sec >> 8U) & 0xFFU);
  out_frame->data[2] = (uint8_t)((uptime_sec >> 16U) & 0xFFU);
  out_frame->data[3] = (uint8_t)((uptime_sec >> 24U) & 0xFFU);
  out_frame->data[4] = (uint8_t)(((health & 0x03U) << 6U) | ((mode & 0x07U) << 3U));
  out_frame->data[5] = 0U;
  out_frame->data[6] = 0U;
  out_frame->data[7] = (uint8_t)(0xC0U | tid);
}

static inline uint8_t dc_encode_tunnel_payload(
    uint8_t protocol,
    uint8_t target_node,
    int8_t serial_id,
    uint8_t options,
    uint32_t baudrate,
    const uint8_t *data,
    uint8_t data_len,
    uint8_t out_payload[DC_TUNNEL_MAX_PAYLOAD]) {
  uint8_t clamped = data_len > DC_TUNNEL_MAX_DATA ? DC_TUNNEL_MAX_DATA : data_len;
  uint8_t sid5 = (uint8_t)(serial_id & 0x1F);

  out_payload[0] = protocol;
  out_payload[1] = (uint8_t)(((target_node & 0x7FU) << 1U) | ((sid5 >> 4U) & 0x01U));
  out_payload[2] = (uint8_t)(((sid5 & 0x0FU) << 4U) | (options & 0x0FU));
  out_payload[3] = (uint8_t)(baudrate & 0xFFU);
  out_payload[4] = (uint8_t)((baudrate >> 8U) & 0xFFU);
  out_payload[5] = (uint8_t)((baudrate >> 16U) & 0xFFU);
  if (clamped > 0U && data != NULL) {
    memcpy(&out_payload[DC_TUNNEL_HEADER_LEN], data, clamped);
  }
  return (uint8_t)(DC_TUNNEL_HEADER_LEN + clamped);
}

static inline bool dc_decode_tunnel_payload(
    const uint8_t *payload,
    size_t payload_len,
    uint8_t *out_protocol,
    uint8_t *out_target_node,
    int8_t *out_serial_id,
    uint8_t *out_options,
    uint32_t *out_baudrate,
    const uint8_t **out_data,
    uint8_t *out_data_len) {
  if (payload == NULL || payload_len < DC_TUNNEL_HEADER_LEN || payload_len > DC_TUNNEL_MAX_PAYLOAD) {
    return false;
  }
  uint8_t sid5 = (uint8_t)(((payload[1] & 0x01U) << 4U) | ((payload[2] >> 4U) & 0x0FU));
  int8_t signed_sid = (sid5 & 0x10U) ? (int8_t)(sid5 | 0xE0U) : (int8_t)sid5;

  if (out_protocol != NULL) {
    *out_protocol = payload[0];
  }
  if (out_target_node != NULL) {
    *out_target_node = (uint8_t)((payload[1] >> 1U) & 0x7FU);
  }
  if (out_serial_id != NULL) {
    *out_serial_id = signed_sid;
  }
  if (out_options != NULL) {
    *out_options = (uint8_t)(payload[2] & 0x0FU);
  }
  if (out_baudrate != NULL) {
    *out_baudrate = (uint32_t)payload[3] |
                    ((uint32_t)payload[4] << 8U) |
                    ((uint32_t)payload[5] << 16U);
  }
  if (out_data != NULL) {
    *out_data = (payload_len > DC_TUNNEL_HEADER_LEN) ? &payload[DC_TUNNEL_HEADER_LEN] : NULL;
  }
  if (out_data_len != NULL) {
    *out_data_len = (uint8_t)(payload_len - DC_TUNNEL_HEADER_LEN);
  }
  return true;
}

static inline size_t dc_frame_transfer(
    uint8_t priority,
    uint16_t dtid,
    uint64_t signature,
    uint8_t src_node,
    const uint8_t *payload,
    size_t payload_len,
    uint8_t *inout_tid,
    DcCanFrame *out_frames,
    size_t max_frames) {
  if (payload == NULL || out_frames == NULL || inout_tid == NULL || payload_len == 0U || max_frames == 0U) {
    return 0U;
  }

  uint32_t can_id = dc_make_broadcast_id(priority, dtid, src_node);
  uint8_t tid = (uint8_t)((*inout_tid) & 0x1FU);

  if (payload_len <= 7U) {
    DcCanFrame *f = &out_frames[0];
    memset(f, 0, sizeof(*f));
    f->id = can_id;
    f->dlc = (uint8_t)(payload_len + 1U);
    memcpy(f->data, payload, payload_len);
    f->data[payload_len] = (uint8_t)(0xC0U | tid);
    *inout_tid = (uint8_t)((tid + 1U) & 0x1FU);
    return 1U;
  }

  uint16_t crc = dc_crc16_init(signature);
  crc = dc_crc16_update(crc, payload, payload_len);

  size_t frame_idx = 0U;
  size_t offset = 0U;
  uint8_t toggle = 0U;

  /* First frame: 2-byte LE CRC + first 5 payload bytes + SOT tail */
  DcCanFrame *f0 = &out_frames[frame_idx++];
  memset(f0, 0, sizeof(*f0));
  f0->id = can_id;
  f0->dlc = 8U;
  f0->data[0] = (uint8_t)(crc & 0xFFU);
  f0->data[1] = (uint8_t)((crc >> 8U) & 0xFFU);
  memcpy(&f0->data[2], payload, 5U);
  f0->data[7] = (uint8_t)(0x80U | tid);
  offset = 5U;
  toggle = 1U;

  while (offset < payload_len) {
    if (frame_idx >= max_frames) {
      return 0U;
    }
    size_t rem = payload_len - offset;
    size_t chunk = rem > 7U ? 7U : rem;
    bool is_last = (offset + chunk) == payload_len;

    DcCanFrame *fn = &out_frames[frame_idx++];
    memset(fn, 0, sizeof(*fn));
    fn->id = can_id;
    fn->dlc = (uint8_t)(chunk + 1U);
    memcpy(fn->data, &payload[offset], chunk);
    fn->data[chunk] = (uint8_t)((is_last ? 0x40U : 0x00U) | (toggle ? 0x20U : 0x00U) | tid);

    offset += chunk;
    toggle ^= 1U;
  }

  *inout_tid = (uint8_t)((tid + 1U) & 0x1FU);
  return frame_idx;
}

static inline void dc_tunnel_rx_init(
    DcTunnelRx *rx,
    uint8_t expected_src_node,
    uint8_t my_node_id,
    int8_t expected_serial_id) {
  memset(rx, 0, sizeof(*rx));
  rx->expected_src_node = expected_src_node;
  rx->my_node_id = my_node_id;
  rx->expected_serial_id = expected_serial_id;
}

static inline bool dc_tunnel_rx_deliver(
    DcTunnelRx *rx,
    const uint8_t *payload,
    size_t payload_len,
    uint8_t *out_data,
    uint8_t *out_len) {
  uint8_t proto = 0U;
  uint8_t target_node = 0U;
  int8_t serial_id = 0;
  const uint8_t *data_ptr = NULL;
  uint8_t data_len = 0U;

  if (!dc_decode_tunnel_payload(
          payload, payload_len, &proto, &target_node, &serial_id, NULL, NULL, &data_ptr, &data_len)) {
    rx->rx_frame_errors++;
    return false;
  }
  (void)proto;
  if ((target_node != rx->my_node_id && target_node != 0U) || serial_id != rx->expected_serial_id) {
    return false;
  }
  rx->rx_transfers_ok++;
  rx->rx_bytes += data_len;
  if (data_len > 0U && out_data != NULL && data_ptr != NULL) {
    memcpy(out_data, data_ptr, data_len);
  }
  if (out_len != NULL) {
    *out_len = data_len;
  }
  return true;
}

static inline bool dc_tunnel_rx_feed(
    DcTunnelRx *rx,
    const DcCanFrame *frame,
    uint32_t now_ms,
    uint8_t *out_data,
    uint8_t *out_len) {
  if (rx == NULL || frame == NULL || frame->dlc < 1U || frame->dlc > 8U) {
    return false;
  }
  /* Message frame bit 7 == 0 */
  if ((frame->id & 0x80UL) != 0UL) {
    return false;
  }
  uint16_t dtid = (uint16_t)((frame->id >> 8U) & 0xFFFFUL);
  uint8_t src_node = (uint8_t)(frame->id & 0x7FUL);
  if (dtid != DC_TUNNEL_TARGETTED_ID) {
    return false;
  }
  if (rx->expected_src_node != 0U && src_node != rx->expected_src_node) {
    return false;
  }

  if (rx->in_progress && (uint32_t)(now_ms - rx->start_ms) > DC_TUNNEL_RX_TIMEOUT_MS) {
    rx->in_progress = false;
    rx->rx_frame_errors++;
  }

  uint8_t tail = frame->data[frame->dlc - 1U];
  bool sot = (tail & 0x80U) != 0U;
  bool eot = (tail & 0x40U) != 0U;
  uint8_t toggle = (uint8_t)((tail >> 5U) & 0x01U);
  uint8_t tid = (uint8_t)(tail & 0x1FU);

  if (sot && eot) {
    if (toggle != 0U) {
      rx->rx_frame_errors++;
      return false;
    }
    if (rx->in_progress) {
      rx->in_progress = false;
      rx->rx_frame_errors++;
    }
    return dc_tunnel_rx_deliver(rx, frame->data, (size_t)(frame->dlc - 1U), out_data, out_len);
  }

  if (sot) {
    if (rx->in_progress) {
      rx->rx_frame_errors++;
    }
    if (toggle != 0U || frame->dlc != 8U) {
      rx->in_progress = false;
      rx->rx_frame_errors++;
      return false;
    }
    rx->expected_crc = (uint16_t)frame->data[0] | ((uint16_t)frame->data[1] << 8U);
    memcpy(rx->buf, &frame->data[2], 5U);
    rx->len = 5U;
    rx->tid = tid;
    rx->expected_toggle = 1U;
    rx->start_ms = now_ms;
    rx->in_progress = true;
    return false;
  }

  if (!rx->in_progress) {
    rx->rx_frame_errors++;
    return false;
  }
  if (tid != rx->tid || toggle != rx->expected_toggle) {
    rx->in_progress = false;
    rx->rx_frame_errors++;
    return false;
  }

  uint8_t chunk = (uint8_t)(frame->dlc - 1U);
  if ((size_t)rx->len + (size_t)chunk > sizeof(rx->buf)) {
    rx->in_progress = false;
    rx->rx_frame_errors++;
    return false;
  }
  if (chunk > 0U) {
    memcpy(&rx->buf[rx->len], frame->data, chunk);
    rx->len = (uint16_t)(rx->len + chunk);
  }
  rx->expected_toggle ^= 1U;

  if (!eot) {
    return false;
  }

  rx->in_progress = false;
  uint16_t calc_crc = dc_crc16_init(DC_TUNNEL_TARGETTED_SIGNATURE);
  calc_crc = dc_crc16_update(calc_crc, rx->buf, rx->len);
  if (calc_crc != rx->expected_crc) {
    rx->rx_crc_errors++;
    return false;
  }

  return dc_tunnel_rx_deliver(rx, rx->buf, rx->len, out_data, out_len);
}

#ifdef __cplusplus
}
#endif
