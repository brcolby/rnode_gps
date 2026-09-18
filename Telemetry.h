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
#include "L76KProtocol.h"
#include "TelemetryFrameParser.h"
#include "TelemetryState.h"

HardwareSerial telemetry_gps_serial(1);
SPIClass telemetry_imu_spi(HSPI);
SensorQMI8658 telemetry_imu;

TelemetryState telemetry_state = telemetry_initial_state();
TelemetryFrameBuffer<CMD_L> telemetry_frame;
uint64_t telemetry_last_imu_us = 0;
char telemetry_nmea[TELEMETRY_NMEA_MAX_BYTES];
size_t telemetry_nmea_length = 0;
bool telemetry_nmea_overflow = false;
L76KCasicParser telemetry_casic_parser;
bool telemetry_navx_pending = false;
bool telemetry_navx_setting = false;
uint64_t telemetry_navx_started_us = 0;

#define TELEMETRY_NAVX_TIMEOUT_US 1000000ULL

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

bool telemetry_send_encoded(const uint8_t *payload, size_t length) {
  if (!telemetry_can_write(payload, length)) return false;
  serial_write(FEND);
  serial_write(CMD_TELEMETRY);
  for (size_t i = 0; i < length; i++) escaped_serial_write(payload[i]);
  serial_write(FEND);
  return true;
}

bool telemetry_send(const uint8_t *body, size_t body_length) {
  uint8_t payload[TELEMETRY_MAX_PAYLOAD_BYTES];
  if (body_length + TELEMETRY_CRC_BYTES > sizeof(payload)) return false;
  memcpy(payload, body, body_length);
  size_t length = telemetry_append_crc(payload, body_length, sizeof(payload));
  return length > 0 && telemetry_send_encoded(payload, length);
}

void telemetry_frame_reset() {
  telemetry_frame.reset();
}

void telemetry_frame_push(uint8_t value) {
  telemetry_frame.push_encoded(value);
}

void telemetry_frame_finish() {
  bool valid = telemetry_frame.finish();
  uint8_t response[20];
  if (valid && telemetry_valid_crc(telemetry_frame.data, telemetry_frame.length)) {
    size_t content_length = telemetry_frame.length - TELEMETRY_CRC_BYTES;
    if (content_length >= 2 &&
        telemetry_frame.data[0] == TELEMETRY_PROTOCOL_VERSION &&
        (telemetry_frame.data[1] == TELEMETRY_GNSS_NAVX_QUERY ||
         telemetry_frame.data[1] == TELEMETRY_GNSS_DYN_MODEL_SET)) {
      bool setting = telemetry_frame.data[1] == TELEMETRY_GNSS_DYN_MODEL_SET;
      size_t expected_length = setting ? 3 : 2;
      if (content_length != expected_length ||
          (setting && telemetry_frame.data[2] > 7)) {
        size_t response_length = telemetry_state_error(
          TELEMETRY_ERROR_VALUE, response, sizeof(response)
        );
        if (response_length > 0) telemetry_send_encoded(response, response_length);
      } else if (telemetry_navx_pending) {
        uint8_t body[] = {TELEMETRY_PROTOCOL_VERSION, TELEMETRY_GNSS_NAVX_STATE, TELEMETRY_NAVX_BUSY};
        telemetry_send(body, sizeof(body));
      } else {
        telemetry_navx_pending = true;
        telemetry_navx_setting = setting;
        telemetry_navx_started_us = (uint64_t)esp_timer_get_time();
        if (setting) {
          uint8_t frame[54];
          size_t frame_length = l76k_build_navx_dynamic_model_set(
            telemetry_frame.data[2], frame, sizeof(frame)
          );
          telemetry_gps_serial.write(frame, frame_length);
        } else {
          telemetry_gps_serial.write(L76K_CFG_NAVX_QUERY, sizeof(L76K_CFG_NAVX_QUERY));
        }
      }
      return;
    }
  }
  size_t response_length = telemetry_process_command(
    telemetry_state,
    telemetry_frame.data,
    telemetry_frame.length,
    !valid,
    MAJ_VERS,
    MIN_VERS,
    response,
    sizeof(response)
  );
  if (response_length > 0) telemetry_send_encoded(response, response_length);
}

void telemetry_emit_navx(uint8_t status, const uint8_t *payload = NULL, size_t length = 0) {
  uint8_t body[3 + L76K_NAVX_PAYLOAD_BYTES];
  if (length > L76K_NAVX_PAYLOAD_BYTES) length = L76K_NAVX_PAYLOAD_BYTES;
  size_t offset = 0;
  body[offset++] = TELEMETRY_PROTOCOL_VERSION;
  body[offset++] = telemetry_navx_setting ? TELEMETRY_GNSS_DYN_MODEL_STATE : TELEMETRY_GNSS_NAVX_STATE;
  body[offset++] = status;
  if (payload != NULL && length > 0) {
    memcpy(body + offset, payload, length);
    offset += length;
  }
  telemetry_send(body, offset);
  telemetry_navx_pending = false;
  telemetry_navx_setting = false;
}

void telemetry_observe_casic(L76KCasicEvent event) {
  if (!telemetry_navx_pending || event == L76K_CASIC_NONE) return;
  if (event == L76K_CASIC_BAD_FRAME) {
    telemetry_emit_navx(TELEMETRY_NAVX_BAD_FRAME);
    return;
  }
  if (!telemetry_navx_setting &&
      telemetry_casic_parser.message_class == L76K_CASIC_CLASS_CFG &&
      telemetry_casic_parser.message_id == L76K_CASIC_ID_NAVX &&
      telemetry_casic_parser.length == L76K_NAVX_PAYLOAD_BYTES) {
    telemetry_emit_navx(
      TELEMETRY_NAVX_OK,
      telemetry_casic_parser.payload,
      telemetry_casic_parser.length
    );
  } else if (telemetry_casic_parser.message_class == L76K_CASIC_CLASS_ACK &&
             telemetry_casic_parser.message_id == L76K_CASIC_ID_ACK &&
             telemetry_casic_parser.length >= 2 &&
             telemetry_casic_parser.payload[0] == L76K_CASIC_CLASS_CFG &&
             telemetry_casic_parser.payload[1] == L76K_CASIC_ID_NAVX) {
    telemetry_emit_navx(TELEMETRY_NAVX_OK);
  } else if (telemetry_casic_parser.message_class == L76K_CASIC_CLASS_ACK &&
             telemetry_casic_parser.message_id == L76K_CASIC_ID_NACK &&
             telemetry_casic_parser.length >= 2 &&
             telemetry_casic_parser.payload[0] == L76K_CASIC_CLASS_CFG &&
             telemetry_casic_parser.payload[1] == L76K_CASIC_ID_NAVX) {
    telemetry_emit_navx(TELEMETRY_NAVX_NACK);
  }
}

void telemetry_emit_nmea() {
  if (!telemetry_valid_nmea(telemetry_nmea, telemetry_nmea_length)) {
    telemetry_state.invalid_nmea++;
    return;
  }
  if (!(telemetry_state.enabled & TELEMETRY_GPS_ENABLED)) return;

  uint8_t payload[2 + 2 + 8 + TELEMETRY_NMEA_MAX_BYTES];
  size_t offset = 0;
  payload[offset++] = TELEMETRY_PROTOCOL_VERSION;
  payload[offset++] = TELEMETRY_GPS_NMEA;
  telemetry_put_u16(payload, offset, telemetry_take_sequence(telemetry_state.gps_sequence));
  telemetry_put_u64(payload, offset, (uint64_t)esp_timer_get_time());
  memcpy(payload + offset, telemetry_nmea, telemetry_nmea_length);
  offset += telemetry_nmea_length;

  telemetry_record_send(telemetry_send(payload, offset), telemetry_state.gps_drops);
}

void telemetry_drain_gps() {
  while (telemetry_gps_serial.available()) {
    uint8_t raw_value = (uint8_t)telemetry_gps_serial.read();
    L76KCasicEvent casic_event;
    if (telemetry_casic_parser.consume(raw_value, casic_event)) {
      telemetry_observe_casic(casic_event);
      continue;
    }
    char value = (char)raw_value;
    if (value == '\n') {
      if (!telemetry_nmea_overflow && telemetry_nmea_length > 0) telemetry_emit_nmea();
      else if (telemetry_nmea_overflow) telemetry_state.invalid_nmea++;
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
    telemetry_state.imu_drops++;
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
  telemetry_put_u16(payload, offset, telemetry_take_sequence(telemetry_state.imu_sequence));
  telemetry_put_u64(payload, offset, timestamp_us);
  telemetry_put_i16(payload, offset, (int16_t)lroundf(ax * 1000.0f));
  telemetry_put_i16(payload, offset, (int16_t)lroundf(ay * 1000.0f));
  telemetry_put_i16(payload, offset, (int16_t)lroundf(az * 1000.0f));
  telemetry_put_i32(payload, offset, (int32_t)lroundf(gx * 1000.0f));
  telemetry_put_i32(payload, offset, (int32_t)lroundf(gy * 1000.0f));
  telemetry_put_i32(payload, offset, (int32_t)lroundf(gz * 1000.0f));
  telemetry_put_i16(payload, offset, (int16_t)lroundf(temperature * 100.0f));
  payload[offset++] = status;

  telemetry_record_send(telemetry_send(payload, offset), telemetry_state.imu_drops);
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
  telemetry_state.ready |= TELEMETRY_GPS_READY;

  // The QMI8658 shares the board's peripheral SPI pins with the SD socket.
  // Keep the unused SD device deselected while the IMU owns transactions.
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  bool imu_ready = telemetry_imu.begin(
    telemetry_imu_spi, IMU_CS, SD_MOSI, SD_MISO, SD_CLK
  );
  if (imu_ready) {
    telemetry_state.ready |= TELEMETRY_IMU_READY;
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
  if (telemetry_navx_pending &&
      (uint64_t)esp_timer_get_time() - telemetry_navx_started_us >= TELEMETRY_NAVX_TIMEOUT_US) {
    telemetry_emit_navx(TELEMETRY_NAVX_TIMEOUT);
  }
  if (!(telemetry_state.enabled & TELEMETRY_IMU_ENABLED) ||
      !(telemetry_state.ready & TELEMETRY_IMU_READY)) return;

  uint64_t now_us = (uint64_t)esp_timer_get_time();
  if (!telemetry_imu_due(now_us, telemetry_last_imu_us, telemetry_state.imu_rate_hz)) return;
  if (!telemetry_imu.getDataReady()) return;

  telemetry_last_imu_us = now_us;
  telemetry_emit_imu(now_us);
}

#else

inline void telemetry_init() {}
inline void telemetry_update() {}
inline void telemetry_frame_reset() {}
inline void telemetry_frame_push(uint8_t) {}
inline void telemetry_frame_finish() {}

#endif

#endif
