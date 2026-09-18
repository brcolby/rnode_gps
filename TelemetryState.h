// Copyright (C) 2026 rnode_gps contributors

#ifndef RNODE_GPS_TELEMETRY_STATE_H
#define RNODE_GPS_TELEMETRY_STATE_H

#include "TelemetryProtocol.h"

struct TelemetryState {
  uint8_t ready;
  uint8_t enabled;
  uint8_t imu_rate_hz;
  uint16_t gps_sequence;
  uint16_t imu_sequence;
  uint32_t invalid_nmea;
  uint32_t gps_drops;
  uint32_t imu_drops;
};

inline TelemetryState telemetry_initial_state() {
  const TelemetryState state = {0, 0, 100, 0, 0, 0, 0, 0};
  return state;
}

inline void telemetry_state_put_u16(uint8_t *buffer, size_t &offset, uint16_t value) {
  buffer[offset++] = value >> 8;
  buffer[offset++] = value;
}

inline void telemetry_state_put_u32(uint8_t *buffer, size_t &offset, uint32_t value) {
  buffer[offset++] = value >> 24;
  buffer[offset++] = value >> 16;
  buffer[offset++] = value >> 8;
  buffer[offset++] = value;
}

inline bool telemetry_valid_rate(uint8_t rate) {
  return rate == 10 || rate == 25 || rate == 50 || rate == 100;
}

inline uint16_t telemetry_take_sequence(uint16_t &sequence) {
  uint16_t result = sequence;
  sequence++;
  return result;
}

inline void telemetry_record_send(bool sent, uint32_t &drops) {
  if (!sent) drops++;
}

inline bool telemetry_imu_due(uint64_t now_us, uint64_t last_us, uint8_t rate_hz) {
  return telemetry_valid_rate(rate_hz) && now_us - last_us >= 1000000ULL / rate_hz;
}

inline int telemetry_nmea_hex(char value) {
  if (value >= '0' && value <= '9') return value - '0';
  if (value >= 'A' && value <= 'F') return value - 'A' + 10;
  if (value >= 'a' && value <= 'f') return value - 'a' + 10;
  return -1;
}

inline bool telemetry_valid_nmea(const char *line, size_t length) {
  if (length < 7 || length > TELEMETRY_NMEA_MAX_BYTES || (line[0] != '$' && line[0] != '!')) {
    return false;
  }
  size_t checksum_at = 0;
  for (size_t i = 1; i < length; i++) {
    if (line[i] == '*') {
      checksum_at = i;
      break;
    }
  }
  if (checksum_at == 0 || checksum_at + 3 != length) return false;

  uint8_t checksum = 0;
  for (size_t i = 1; i < checksum_at; i++) checksum ^= (uint8_t)line[i];
  int high = telemetry_nmea_hex(line[checksum_at + 1]);
  int low = telemetry_nmea_hex(line[checksum_at + 2]);
  return high >= 0 && low >= 0 && checksum == (uint8_t)((high << 4) | low);
}

inline size_t telemetry_state_response(
  uint8_t subtype,
  const uint8_t *body,
  size_t body_length,
  uint8_t *response,
  size_t capacity
) {
  if (2 + body_length + TELEMETRY_CRC_BYTES > capacity) return 0;
  size_t offset = 0;
  response[offset++] = TELEMETRY_PROTOCOL_VERSION;
  response[offset++] = subtype;
  for (size_t i = 0; i < body_length; i++) response[offset++] = body[i];
  return telemetry_append_crc(response, offset, capacity);
}

inline size_t telemetry_state_error(uint8_t code, uint8_t *response, size_t capacity) {
  return telemetry_state_response(TELEMETRY_ERROR, &code, 1, response, capacity);
}

inline size_t telemetry_process_command(
  TelemetryState &state,
  const uint8_t *payload,
  size_t length,
  bool malformed,
  uint8_t firmware_major,
  uint8_t firmware_minor,
  uint8_t *response,
  size_t capacity
) {
  if (malformed || !telemetry_valid_crc(payload, length)) {
    return telemetry_state_error(TELEMETRY_ERROR_INTEGRITY, response, capacity);
  }
  size_t content_length = length - TELEMETRY_CRC_BYTES;
  if (payload[0] != TELEMETRY_PROTOCOL_VERSION) {
    return telemetry_state_error(TELEMETRY_ERROR_VERSION, response, capacity);
  }

  switch (payload[1]) {
    case TELEMETRY_CAPS_QUERY: {
      if (content_length != 2) {
        return telemetry_state_error(TELEMETRY_ERROR_VALUE, response, capacity);
      }
      const uint8_t body[] = {
        TELEMETRY_GPS_ENABLED | TELEMETRY_IMU_ENABLED,
        state.ready,
        0x0F,
        firmware_major,
        firmware_minor,
      };
      return telemetry_state_response(TELEMETRY_CAPS_RESPONSE, body, sizeof(body), response, capacity);
    }

    case TELEMETRY_CONFIG_SET: {
      if (content_length != 4 || !telemetry_valid_rate(payload[3])) {
        return telemetry_state_error(TELEMETRY_ERROR_VALUE, response, capacity);
      }
      state.enabled = payload[2] & state.ready & (TELEMETRY_GPS_ENABLED | TELEMETRY_IMU_ENABLED);
      state.imu_rate_hz = payload[3];
      const uint8_t body[] = {state.enabled, state.imu_rate_hz, state.ready};
      return telemetry_state_response(TELEMETRY_CONFIG_STATE, body, sizeof(body), response, capacity);
    }

    case TELEMETRY_STATS_QUERY: {
      if (content_length != 2) {
        return telemetry_state_error(TELEMETRY_ERROR_VALUE, response, capacity);
      }
      uint8_t body[16];
      size_t offset = 0;
      telemetry_state_put_u16(body, offset, state.gps_sequence);
      telemetry_state_put_u16(body, offset, state.imu_sequence);
      telemetry_state_put_u32(body, offset, state.invalid_nmea);
      telemetry_state_put_u32(body, offset, state.gps_drops);
      telemetry_state_put_u32(body, offset, state.imu_drops);
      return telemetry_state_response(TELEMETRY_STATS_RESPONSE, body, offset, response, capacity);
    }

    default:
      return telemetry_state_error(TELEMETRY_ERROR_SUBTYPE, response, capacity);
  }
}

#endif
