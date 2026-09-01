// Copyright (C) 2026 rnode_gps contributors
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#ifndef RNODE_GPS_TELEMETRY_H
#define RNODE_GPS_TELEMETRY_H

#if defined(RNODE_GPS_TELEMETRY) && BOARD_MODEL == BOARD_TBEAM_S_V1

#include <HardwareSerial.h>
#include <SensorQMI8658.hpp>
#include <esp_timer.h>
#include <math.h>
#include "TelemetryProtocol.h"

HardwareSerial telemetry_gps_serial(1);
SPIClass telemetry_imu_spi(HSPI);
SensorQMI8658 telemetry_imu;

bool telemetry_gps_ready = false;
bool telemetry_imu_ready = false;
uint8_t telemetry_enabled = 0;
uint8_t telemetry_imu_rate_hz = 50;
uint16_t telemetry_gps_sequence = 0;
uint16_t telemetry_imu_sequence = 0;
uint32_t telemetry_invalid_nmea = 0;
uint32_t telemetry_gps_drops = 0;
uint32_t telemetry_imu_drops = 0;
uint64_t telemetry_last_imu_us = 0;
char telemetry_nmea[TELEMETRY_NMEA_MAX_BYTES];
size_t telemetry_nmea_length = 0;
bool telemetry_nmea_overflow = false;

inline void telemetry_put_u16(uint8_t *buffer, size_t &offset, uint16_t value) {
  buffer[offset++] = value >> 8;
  buffer[offset++] = value;
}

inline void telemetry_put_i16(uint8_t *buffer, size_t &offset, int16_t value) {
  telemetry_put_u16(buffer, offset, (uint16_t)value);
}

inline void telemetry_put_u32(uint8_t *buffer, size_t &offset, uint32_t value) {
  buffer[offset++] = value >> 24;
  buffer[offset++] = value >> 16;
  buffer[offset++] = value >> 8;
  buffer[offset++] = value;
}

inline void telemetry_put_i32(uint8_t *buffer, size_t &offset, int32_t value) {
  telemetry_put_u32(buffer, offset, (uint32_t)value);
}

inline void telemetry_put_u64(uint8_t *buffer, size_t &offset, uint64_t value) {
  for (int shift = 56; shift >= 0; shift -= 8) {
    buffer[offset++] = value >> shift;
  }
}

bool telemetry_can_write(const uint8_t *payload, size_t length) {
  size_t encoded_length = 3; // two delimiters and the command byte
  for (size_t i = 0; i < length; i++) {
    encoded_length += (payload[i] == FEND || payload[i] == FESC) ? 2 : 1;
  }
  return Serial.availableForWrite() >= encoded_length;
}

bool telemetry_send(const uint8_t *body, size_t body_length) {
  uint8_t payload[TELEMETRY_MAX_PAYLOAD_BYTES];
  if (body_length + TELEMETRY_CRC_BYTES > sizeof(payload)) return false;
  memcpy(payload, body, body_length);
  size_t length = telemetry_append_crc(payload, body_length, sizeof(payload));
  if (length == 0 || !telemetry_can_write(payload, length)) return false;

  serial_write(FEND);
  serial_write(CMD_TELEMETRY);
  for (size_t i = 0; i < length; i++) escaped_serial_write(payload[i]);
  serial_write(FEND);
  return true;
}

uint8_t telemetry_status() {
  uint8_t status = 0;
  if (telemetry_gps_ready) status |= TELEMETRY_GPS_READY;
  if (telemetry_imu_ready) status |= TELEMETRY_IMU_READY;
  return status;
}

void telemetry_send_caps() {
  const uint8_t payload[] = {
    TELEMETRY_PROTOCOL_VERSION,
    TELEMETRY_CAPS_RESPONSE,
    TELEMETRY_GPS_ENABLED | TELEMETRY_IMU_ENABLED,
    telemetry_status(),
    0x0F, // supported rates: 10, 25, 50 and 100 Hz
    MAJ_VERS,
    MIN_VERS,
  };
  telemetry_send(payload, sizeof(payload));
}

void telemetry_send_config_state() {
  const uint8_t payload[] = {
    TELEMETRY_PROTOCOL_VERSION,
    TELEMETRY_CONFIG_STATE,
    telemetry_enabled,
    telemetry_imu_rate_hz,
    telemetry_status(),
  };
  telemetry_send(payload, sizeof(payload));
}

void telemetry_send_error(uint8_t code) {
  const uint8_t payload[] = {
    TELEMETRY_PROTOCOL_VERSION,
    TELEMETRY_ERROR,
    code,
  };
  telemetry_send(payload, sizeof(payload));
}

void telemetry_send_stats() {
  uint8_t payload[20];
  size_t offset = 0;
  payload[offset++] = TELEMETRY_PROTOCOL_VERSION;
  payload[offset++] = TELEMETRY_STATS_RESPONSE;
  telemetry_put_u16(payload, offset, telemetry_gps_sequence);
  telemetry_put_u16(payload, offset, telemetry_imu_sequence);
  telemetry_put_u32(payload, offset, telemetry_invalid_nmea);
  telemetry_put_u32(payload, offset, telemetry_gps_drops);
  telemetry_put_u32(payload, offset, telemetry_imu_drops);
  telemetry_send(payload, offset);
}

bool telemetry_valid_rate(uint8_t rate) {
  return rate == 10 || rate == 25 || rate == 50 || rate == 100;
}

void telemetry_handle_command(const uint8_t *payload, size_t length, bool malformed = false) {
  if (malformed || !telemetry_valid_crc(payload, length)) {
    telemetry_send_error(TELEMETRY_ERROR_INTEGRITY);
    return;
  }
  length -= TELEMETRY_CRC_BYTES;
  if (payload[0] != TELEMETRY_PROTOCOL_VERSION) {
    telemetry_send_error(TELEMETRY_ERROR_VERSION);
    return;
  }

  switch (payload[1]) {
    case TELEMETRY_CAPS_QUERY:
      if (length == 2) telemetry_send_caps();
      else telemetry_send_error(TELEMETRY_ERROR_VALUE);
      break;

    case TELEMETRY_CONFIG_SET:
      if (length != 4 || !telemetry_valid_rate(payload[3])) {
        telemetry_send_error(TELEMETRY_ERROR_VALUE);
        break;
      }
      telemetry_enabled = payload[2] & (TELEMETRY_GPS_ENABLED | TELEMETRY_IMU_ENABLED);
      if (!telemetry_gps_ready) telemetry_enabled &= ~TELEMETRY_GPS_ENABLED;
      if (!telemetry_imu_ready) telemetry_enabled &= ~TELEMETRY_IMU_ENABLED;
      telemetry_imu_rate_hz = payload[3];
      telemetry_send_config_state();
      break;

    case TELEMETRY_STATS_QUERY:
      if (length == 2) telemetry_send_stats();
      else telemetry_send_error(TELEMETRY_ERROR_VALUE);
      break;

    default:
      telemetry_send_error(TELEMETRY_ERROR_SUBTYPE);
      break;
  }
}

int telemetry_hex(char value) {
  if (value >= '0' && value <= '9') return value - '0';
  if (value >= 'A' && value <= 'F') return value - 'A' + 10;
  if (value >= 'a' && value <= 'f') return value - 'a' + 10;
  return -1;
}

bool telemetry_valid_nmea(const char *line, size_t length) {
  if (length < 7 || (line[0] != '$' && line[0] != '!')) return false;

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
  int high = telemetry_hex(line[checksum_at + 1]);
  int low = telemetry_hex(line[checksum_at + 2]);
  return high >= 0 && low >= 0 && checksum == (uint8_t)((high << 4) | low);
}

void telemetry_emit_nmea() {
  if (!telemetry_valid_nmea(telemetry_nmea, telemetry_nmea_length)) {
    telemetry_invalid_nmea++;
    return;
  }
  if (!(telemetry_enabled & TELEMETRY_GPS_ENABLED)) return;

  uint8_t payload[2 + 2 + 8 + TELEMETRY_NMEA_MAX_BYTES];
  size_t offset = 0;
  payload[offset++] = TELEMETRY_PROTOCOL_VERSION;
  payload[offset++] = TELEMETRY_GPS_NMEA;
  telemetry_put_u16(payload, offset, telemetry_gps_sequence);
  telemetry_put_u64(payload, offset, (uint64_t)esp_timer_get_time());
  memcpy(payload + offset, telemetry_nmea, telemetry_nmea_length);
  offset += telemetry_nmea_length;

  telemetry_gps_sequence++;
  if (!telemetry_send(payload, offset)) telemetry_gps_drops++;
}

void telemetry_drain_gps() {
  while (telemetry_gps_serial.available()) {
    char value = (char)telemetry_gps_serial.read();
    if (value == '\n') {
      if (!telemetry_nmea_overflow && telemetry_nmea_length > 0) telemetry_emit_nmea();
      else if (telemetry_nmea_overflow) telemetry_invalid_nmea++;
      telemetry_nmea_length = 0;
      telemetry_nmea_overflow = false;
    } else if (value != '\r') {
      if (telemetry_nmea_length < sizeof(telemetry_nmea)) {
        telemetry_nmea[telemetry_nmea_length++] = value;
      } else {
        telemetry_nmea_overflow = true;
      }
    }
  }
}

void telemetry_emit_imu(uint64_t timestamp_us) {
  float ax, ay, az, gx, gy, gz;
  if (!telemetry_imu.getAccelerometer(ax, ay, az) ||
      !telemetry_imu.getGyroscope(gx, gy, gz)) {
    telemetry_imu_drops++;
    return;
  }

  float temperature = telemetry_imu.getTemperature_C();
  uint8_t status = 0x03;
  if (isfinite(temperature)) status |= 0x04;
  else temperature = 0.0f;

  uint8_t payload[33];
  size_t offset = 0;
  payload[offset++] = TELEMETRY_PROTOCOL_VERSION;
  payload[offset++] = TELEMETRY_IMU_SAMPLE;
  telemetry_put_u16(payload, offset, telemetry_imu_sequence);
  telemetry_put_u64(payload, offset, timestamp_us);
  telemetry_put_i16(payload, offset, (int16_t)lroundf(ax * 1000.0f));
  telemetry_put_i16(payload, offset, (int16_t)lroundf(ay * 1000.0f));
  telemetry_put_i16(payload, offset, (int16_t)lroundf(az * 1000.0f));
  telemetry_put_i32(payload, offset, (int32_t)lroundf(gx * 1000.0f));
  telemetry_put_i32(payload, offset, (int32_t)lroundf(gy * 1000.0f));
  telemetry_put_i32(payload, offset, (int32_t)lroundf(gz * 1000.0f));
  telemetry_put_i16(payload, offset, (int16_t)lroundf(temperature * 100.0f));
  payload[offset++] = status;

  telemetry_imu_sequence++;
  if (!telemetry_send(payload, offset)) telemetry_imu_drops++;
}

void telemetry_init() {
  // LilyGO drives GPIO 7 high to wake the L76K. It is unconnected on the
  // u-blox option, so applying the same level is harmless there. Allow the
  // AXP2101 rails configured by init_pmu() to settle before probing sensors.
  pinMode(PIN_GPS_WAKE, OUTPUT);
  digitalWrite(PIN_GPS_WAKE, HIGH);
  delay(250);

  telemetry_gps_serial.setRxBufferSize(1024);
  telemetry_gps_serial.begin(GPS_BAUD_RATE, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  telemetry_gps_ready = true;

  // The QMI8658 shares the board's peripheral SPI pins with the SD socket.
  // Keep the unused SD device deselected while the IMU owns transactions.
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  telemetry_imu_ready = telemetry_imu.begin(
    telemetry_imu_spi, IMU_CS, SD_MOSI, SD_MISO, SD_CLK
  );
  if (telemetry_imu_ready) {
    telemetry_imu.configAccelerometer(
      SensorQMI8658::ACC_RANGE_4G,
      SensorQMI8658::ACC_ODR_125Hz,
      SensorQMI8658::LPF_MODE_0
    );
    telemetry_imu.configGyroscope(
      SensorQMI8658::GYR_RANGE_512DPS,
      SensorQMI8658::GYR_ODR_112_1Hz,
      SensorQMI8658::LPF_MODE_3
    );
    telemetry_imu.enableGyroscope();
    telemetry_imu.enableAccelerometer();
  }
}

void telemetry_update() {
  telemetry_drain_gps();
  if (!(telemetry_enabled & TELEMETRY_IMU_ENABLED) || !telemetry_imu_ready) return;

  uint64_t now_us = (uint64_t)esp_timer_get_time();
  uint64_t interval_us = 1000000ULL / telemetry_imu_rate_hz;
  if (now_us - telemetry_last_imu_us < interval_us) return;
  if (!telemetry_imu.getDataReady()) return;

  telemetry_last_imu_us = now_us;
  telemetry_emit_imu(now_us);
}

#else

inline void telemetry_init() {}
inline void telemetry_update() {}
inline void telemetry_handle_command(const uint8_t *, size_t, bool = false) {}

#endif

#endif
