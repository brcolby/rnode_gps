// Copyright (C) 2026 rnode_gps contributors
//
// Private, compile-time-independent wire primitives shared by the firmware
// implementation and host-side protocol conformance tests.

#ifndef RNODE_GPS_TELEMETRY_PROTOCOL_H
#define RNODE_GPS_TELEMETRY_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#define TELEMETRY_PROTOCOL_VERSION 0x01

#define TELEMETRY_CAPS_QUERY        0x00
#define TELEMETRY_CAPS_RESPONSE     0x01
#define TELEMETRY_CONFIG_SET        0x02
#define TELEMETRY_CONFIG_STATE      0x03
#define TELEMETRY_STATS_QUERY       0x04
#define TELEMETRY_STATS_RESPONSE    0x05
#define TELEMETRY_GPS_NMEA          0x10
#define TELEMETRY_IMU_SAMPLE        0x20
#define TELEMETRY_ERROR             0x7F

#define TELEMETRY_GPS_ENABLED       0x01
#define TELEMETRY_IMU_ENABLED       0x02
#define TELEMETRY_GPS_READY         0x01
#define TELEMETRY_IMU_READY         0x02

#define TELEMETRY_ERROR_VERSION     0x01
#define TELEMETRY_ERROR_VALUE       0x02
#define TELEMETRY_ERROR_SUBTYPE     0x03
#define TELEMETRY_ERROR_INTEGRITY   0x04

#define TELEMETRY_NMEA_MAX_BYTES    128
#define TELEMETRY_CRC_BYTES         2
#define TELEMETRY_MAX_PAYLOAD_BYTES (2 + 2 + 8 + TELEMETRY_NMEA_MAX_BYTES + TELEMETRY_CRC_BYTES)

// CRC-16/CCITT-FALSE: polynomial 0x1021, initial value 0xffff, no reflection,
// no final xor. The CRC covers version, subtype, and body, and is appended
// most-significant byte first.
inline uint16_t telemetry_crc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

inline bool telemetry_valid_crc(const uint8_t *payload, size_t length) {
  if (length < 2 + TELEMETRY_CRC_BYTES) return false;
  size_t body_length = length - TELEMETRY_CRC_BYTES;
  uint16_t expected = (uint16_t)payload[body_length] << 8 | payload[body_length + 1];
  return telemetry_crc16(payload, body_length) == expected;
}

inline size_t telemetry_append_crc(uint8_t *payload, size_t length, size_t capacity) {
  if (length + TELEMETRY_CRC_BYTES > capacity) return 0;
  uint16_t crc = telemetry_crc16(payload, length);
  payload[length++] = crc >> 8;
  payload[length++] = crc;
  return length;
}

#endif
