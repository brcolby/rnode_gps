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
#include <atomic>
#include "L76KProtocol.h"
#include "TelemetryFrameParser.h"
#include "TelemetryState.h"
#include "TelemetryImu.h"

HardwareSerial telemetry_gps_serial(1);
SPIClass telemetry_imu_spi(HSPI);
SensorQMI8658 telemetry_imu;

TelemetryState telemetry_state = telemetry_initial_state();
TelemetryFrameBuffer<CMD_L> telemetry_frame;
struct TelemetryImuSample { uint8_t payload[TELEMETRY_IMU_CAPTURE_BYTES]; };
QueueHandle_t telemetry_imu_queue = NULL;
std::atomic<uint8_t> telemetry_imu_task_rate(0);
std::atomic<uint16_t> telemetry_imu_task_sequence(0);
std::atomic<uint32_t> telemetry_imu_task_drops(0);
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
  #if defined(RNODE_GPS_HOST_UART)
    uint8_t wire[TELEMETRY_MAX_WIRE_BYTES];
    size_t wire_length = telemetry_encode_kiss(CMD_TELEMETRY, payload, length, wire, sizeof(wire));
    if (!wire_length || Serial.availableForWrite() < wire_length) return false;
    return Serial.write(wire, wire_length) == wire_length;
  #else
  if (!telemetry_can_write(payload, length)) return false;
  serial_write(FEND);
  serial_write(CMD_TELEMETRY);
  for (size_t i = 0; i < length; i++) escaped_serial_write(payload[i]);
  serial_write(FEND);
  return true;
  #endif
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

// Read the entire locked TIMESTAMP..GZ_H register block in one transaction.
// GZ_H releases the lock. SensorLib 0.3.3's getTimestamp() has incorrect repeat/
// wrap arithmetic, so retain the hardware's raw 24-bit count instead.
void telemetry_read_imu_registers(uint8_t address, uint8_t *data, size_t length) {
  telemetry_imu_spi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(IMU_CS, LOW);
  telemetry_imu_spi.transfer(address | 0x80);
  for (size_t i = 0; i < length; ++i) data[i] = telemetry_imu_spi.transfer(0);
  digitalWrite(IMU_CS, HIGH);
  telemetry_imu_spi.endTransaction();
}

void telemetry_capture_imu(const uint8_t *raw, uint16_t sequence, uint8_t status,
    uint64_t lower_us, uint64_t upper_us, uint64_t read_end_us, TelemetryImuSample &sample) {
  uint32_t counter = raw[0] | (uint32_t)raw[1] << 8 | (uint32_t)raw[2] << 16;
  bool bracketed = telemetry_imu_bracket_valid(lower_us, upper_us, read_end_us);
  // Midpoint of observed data-ready bounds, not UART/host receipt time. The
  // sensor's filter/group delay is separate and must be calibrated on the host.
  uint64_t capture_us = bracketed ? lower_us + (upper_us - lower_us) / 2 : upper_us;
  if (!bracketed) status |= TELEMETRY_IMU_TIMING_FAULT;
  for (int i = 5; i < 17; i += 2) {
    if (telemetry_imu_saturated(telemetry_imu_i16(raw + i))) status |= TELEMETRY_IMU_SATURATED;
  }
  uint8_t *payload = sample.payload;
  size_t offset = 0;
  payload[offset++] = TELEMETRY_PROTOCOL_VERSION;
  payload[offset++] = TELEMETRY_IMU_CAPTURE;
  telemetry_put_u16(payload, offset, sequence);
  telemetry_put_u64(payload, offset, capture_us);
  for (int i = 5; i < 11; i += 2)
    telemetry_put_i16(payload, offset, telemetry_imu_scaled(telemetry_imu_i16(raw + i), 4000));
  for (int i = 11; i < 17; i += 2)
    telemetry_put_i32(payload, offset, telemetry_imu_scaled(telemetry_imu_i16(raw + i), 512000));
  telemetry_put_i16(payload, offset, telemetry_imu_scaled(telemetry_imu_i16(raw + 3), 12800));
  payload[offset++] = status;
  telemetry_put_u32(payload, offset, counter);
  telemetry_put_u64(payload, offset, lower_us);
  telemetry_put_u64(payload, offset, upper_us);
  telemetry_put_u64(payload, offset, read_end_us);
}

void telemetry_imu_worker(void *) {
  uint8_t last_rate = 0;
  uint16_t sequence = 0;
  uint64_t last_empty_us = 0, last_output_us = 0;
  TelemetryImuContinuity continuity;
  for (;;) {
    // Poll at one RTOS tick, not at the requested publication frequency. The
    // 100-Hz target means every native sample (nominal ~112 Hz on this recipe).
    // 10/25/50 are explicitly decimated diagnostics, not lossless VIO inputs.
    vTaskDelay(1);
    uint8_t rate = telemetry_imu_task_rate.load();
    if (!rate) { last_rate = 0; last_empty_us = 0; continue; }
    if (rate != last_rate) {
      continuity = TelemetryImuContinuity();
      continuity.pending_faults = TELEMETRY_IMU_TIMING_FAULT;
      last_empty_us = last_output_us = 0;
      last_rate = rate;
    }
    uint64_t poll_start = (uint64_t)esp_timer_get_time();
    uint8_t status = telemetry_imu.getIrqStatus();
    uint64_t ready_observed = (uint64_t)esp_timer_get_time();
    if (!(status & 0x01)) {
      // Use the start of the SPI read as a conservative lower bound.
      last_empty_us = poll_start;
      continue;
    }
    // QMI locking protocol requires >=12 us at this gyro ODR. Verify Locked
    // rather than assuming a delayed read or failed SPI transaction succeeded.
    delayMicroseconds(20);
    if (!(telemetry_imu.getIrqStatus() & 0x02)) {
      uint8_t ignored;
      telemetry_read_imu_registers(0x40, &ignored, 1); // release any partial lock
      continuity.pending_faults |= TELEMETRY_IMU_TIMING_FAULT | TELEMETRY_IMU_SAMPLE_LOSS;
      telemetry_imu_task_drops.fetch_add(1);
      telemetry_imu_task_sequence.store(++sequence);
      last_empty_us = 0;
      continue;
    }
    uint8_t raw[17];
    telemetry_read_imu_registers(0x30, raw, sizeof(raw));
    uint64_t read_end = (uint64_t)esp_timer_get_time();
    uint32_t counter = raw[0] | (uint32_t)raw[1] << 8 | (uint32_t)raw[2] << 16;
    uint32_t missed = continuity.observe(counter);
    telemetry_imu_task_drops.fetch_add(missed);
    if (rate == 100) sequence += missed;
    if (rate == 100 || read_end - last_output_us >= 1000000ULL / rate) {
      TelemetryImuSample sample;
      uint8_t flags = 0x07 | continuity.pending_faults;
      if (rate != 100) flags |= TELEMETRY_IMU_DECIMATED;
      telemetry_capture_imu(raw, sequence++, flags, last_empty_us, ready_observed, read_end, sample);
      bool sent = xQueueSend(telemetry_imu_queue, &sample, 0) == pdTRUE;
      continuity.delivery(sent);
      if (!sent) telemetry_imu_task_drops.fetch_add(1);
      telemetry_imu_task_sequence.store(sequence);
      last_output_us = read_end;
    }
    // A completed read releases the lock, but does not prove no new sample is
    // ready. Require an actual empty poll before claiming a bounded next epoch.
    last_empty_us = 0;
  }
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
    bool configured = telemetry_imu.configAccelerometer(
      SensorQMI8658::ACC_RANGE_4G,
      SensorQMI8658::ACC_ODR_125Hz,
      SensorQMI8658::LPF_MODE_0
    ) == 0;
    configured &= telemetry_imu.configGyroscope(
      SensorQMI8658::GYR_RANGE_512DPS,
      SensorQMI8658::GYR_ODR_112_1Hz,
      SensorQMI8658::LPF_MODE_3
    ) == 0;
    configured &= telemetry_imu.enableGyroscope();
    configured &= telemetry_imu.enableAccelerometer();
    configured &= telemetry_imu.enableLockingMechanism() == 0;
    if (!configured) return;
    telemetry_imu_queue = xQueueCreate(64, sizeof(TelemetryImuSample));
    if (telemetry_imu_queue && xTaskCreate(telemetry_imu_worker, "rnode-imu", 4096, NULL, 2, NULL) == pdPASS) {
      telemetry_state.ready |= TELEMETRY_IMU_READY;
    }
  }
}

void telemetry_update() {
  telemetry_drain_gps();
  if (telemetry_navx_pending &&
      (uint64_t)esp_timer_get_time() - telemetry_navx_started_us >= TELEMETRY_NAVX_TIMEOUT_US) {
    telemetry_emit_navx(TELEMETRY_NAVX_TIMEOUT);
  }
  const bool enabled = (telemetry_state.enabled & TELEMETRY_IMU_ENABLED) &&
                       (telemetry_state.ready & TELEMETRY_IMU_READY);
  telemetry_imu_task_rate.store(enabled ? telemetry_state.imu_rate_hz : 0);
  telemetry_state.imu_drops += telemetry_imu_task_drops.exchange(0);
  telemetry_state.imu_sequence = telemetry_imu_task_sequence.load();
  if (!telemetry_imu_queue) return;
  TelemetryImuSample sample;
  // Bound per-loop work, leaving radio/control traffic service opportunities.
  for (int count = 0; count < 8 && xQueuePeek(telemetry_imu_queue, &sample, 0) == pdTRUE; ++count) {
    if (enabled && !telemetry_send(sample.payload, sizeof(sample.payload))) break;
    xQueueReceive(telemetry_imu_queue, &sample, 0);
  }
}

#else

inline void telemetry_init() {}
inline void telemetry_update() {}
inline void telemetry_frame_reset() {}
inline void telemetry_frame_push(uint8_t) {}
inline void telemetry_frame_finish() {}

#endif

#endif
