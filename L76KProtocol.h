// Copyright (C) 2026 rnode_gps contributors
//
// Minimal CASIC framing support for querying the L76K navigation engine.

#ifndef RNODE_GPS_L76K_PROTOCOL_H
#define RNODE_GPS_L76K_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define L76K_CASIC_CLASS_ACK 0x05
#define L76K_CASIC_ID_NACK 0x00
#define L76K_CASIC_ID_ACK 0x01
#define L76K_CASIC_CLASS_CFG 0x06
#define L76K_CASIC_ID_NAVX 0x07
#define L76K_NAVX_PAYLOAD_BYTES 44
#define L76K_CASIC_MAX_PAYLOAD_BYTES 64

const uint8_t L76K_CFG_NAVX_QUERY[] = {
  0xBA, 0xCE, 0x00, 0x00, 0x06, 0x07, 0x00, 0x00, 0x06, 0x07,
};

inline size_t l76k_build_navx_dynamic_model_set(uint8_t dynamic_model, uint8_t *frame, size_t capacity) {
  const size_t frame_length = 2 + 2 + 2 + L76K_NAVX_PAYLOAD_BYTES + 4;
  if (capacity < frame_length || dynamic_model > 7) return 0;
  memset(frame, 0, frame_length);
  frame[0] = 0xBA;
  frame[1] = 0xCE;
  frame[2] = L76K_NAVX_PAYLOAD_BYTES;
  frame[4] = L76K_CASIC_CLASS_CFG;
  frame[5] = L76K_CASIC_ID_NAVX;
  frame[6] = 0x01; // mask bit 0: apply only dyModel
  frame[10] = dynamic_model; // payload offset 4

  uint32_t checksum = (uint32_t)L76K_NAVX_PAYLOAD_BYTES |
                      (uint32_t)L76K_CASIC_CLASS_CFG << 16 |
                      (uint32_t)L76K_CASIC_ID_NAVX << 24 |
                      0x00000001UL |
                      (uint32_t)dynamic_model;
  size_t checksum_offset = frame_length - 4;
  frame[checksum_offset] = checksum;
  frame[checksum_offset + 1] = checksum >> 8;
  frame[checksum_offset + 2] = checksum >> 16;
  frame[checksum_offset + 3] = checksum >> 24;
  return frame_length;
}

enum L76KCasicEvent {
  L76K_CASIC_NONE = 0,
  L76K_CASIC_FRAME = 1,
  L76K_CASIC_BAD_FRAME = 2,
};

struct L76KCasicParser {
  uint8_t state;
  uint16_t length;
  uint16_t payload_offset;
  uint8_t message_class;
  uint8_t message_id;
  uint8_t payload[L76K_CASIC_MAX_PAYLOAD_BYTES];
  uint8_t checksum_bytes[4];

  L76KCasicParser() { reset(); }

  void reset() {
    state = 0;
    length = 0;
    payload_offset = 0;
    message_class = 0;
    message_id = 0;
  }

  static uint32_t little_u32(const uint8_t *bytes) {
    return (uint32_t)bytes[0] |
           (uint32_t)bytes[1] << 8 |
           (uint32_t)bytes[2] << 16 |
           (uint32_t)bytes[3] << 24;
  }

  uint32_t expected_checksum() const {
    uint32_t checksum = (uint32_t)length |
                        (uint32_t)message_class << 16 |
                        (uint32_t)message_id << 24;
    for (uint16_t offset = 0; offset < length; offset += 4) {
      checksum += little_u32(payload + offset);
    }
    return checksum;
  }

  // Returns true when the byte belongs to a CASIC frame. Ordinary NMEA bytes
  // return false. event is set only when a complete or invalid frame arrives.
  bool consume(uint8_t value, L76KCasicEvent &event) {
    event = L76K_CASIC_NONE;
    switch (state) {
      case 0:
        if (value != 0xBA) return false;
        state = 1;
        return true;
      case 1:
        if (value == 0xCE) state = 2;
        else if (value != 0xBA) {
          reset();
          return false;
        }
        return true;
      case 2:
        length = value;
        state = 3;
        return true;
      case 3:
        length |= (uint16_t)value << 8;
        if (length > sizeof(payload) || (length & 0x03) != 0) {
          reset();
          event = L76K_CASIC_BAD_FRAME;
        } else {
          state = 4;
        }
        return true;
      case 4:
        message_class = value;
        state = 5;
        return true;
      case 5:
        message_id = value;
        payload_offset = 0;
        state = length == 0 ? 7 : 6;
        return true;
      case 6:
        payload[payload_offset++] = value;
        if (payload_offset == length) state = 7;
        return true;
      default: {
        uint8_t checksum_offset = state - 7;
        checksum_bytes[checksum_offset] = value;
        if (checksum_offset == 3) {
          uint32_t received = little_u32(checksum_bytes);
          state = 0;
          event = received == expected_checksum() ? L76K_CASIC_FRAME : L76K_CASIC_BAD_FRAME;
        } else {
          state++;
        }
        return true;
      }
    }
  }
};

#endif
