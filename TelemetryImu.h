// Copyright (C) 2026 rnode_gps contributors
// Pure acquisition accounting; shared with host-native failure-path tests.
#ifndef RNODE_GPS_TELEMETRY_IMU_H
#define RNODE_GPS_TELEMETRY_IMU_H

#include <stdint.h>

#define TELEMETRY_IMU_SATURATED 0x08
#define TELEMETRY_IMU_SAMPLE_LOSS 0x10
#define TELEMETRY_IMU_QUEUE_OVERFLOW 0x20
#define TELEMETRY_IMU_TIMING_FAULT 0x40
#define TELEMETRY_IMU_DECIMATED 0x80
#define TELEMETRY_IMU_CAPTURE_BYTES 61

struct TelemetryImuContinuity {
  bool initialized;
  uint32_t previous;
  uint8_t pending_faults;
  TelemetryImuContinuity() : initialized(false), previous(0), pending_faults(0) {}

  // The QMI counter is a wrapping 24-bit sample count, NOT microseconds.
  // A duplicate/backwards counter is a fault, never a huge inferred loss count.
  uint32_t observe(uint32_t current) {
    current &= 0xffffff;
    uint32_t missed = 0;
    if (initialized) {
      uint32_t delta = (current - previous) & 0xffffff;
      if (!delta || delta >= 0x800000) pending_faults |= TELEMETRY_IMU_TIMING_FAULT;
      else if (delta != 1) {
        pending_faults |= TELEMETRY_IMU_SAMPLE_LOSS;
        missed = delta - 1;
      }
    }
    previous = current;
    initialized = true;
    return missed;
  }

  void delivery(bool accepted) {
    // Retain all faults until an affected sample actually enters the queue.
    if (accepted) pending_faults = 0;
    else pending_faults |= TELEMETRY_IMU_QUEUE_OVERFLOW | TELEMETRY_IMU_SAMPLE_LOSS;
  }
};

inline int16_t telemetry_imu_i16(const uint8_t *data) {
  return (int16_t)((uint16_t)data[0] | (uint16_t)data[1] << 8);
}

inline int32_t telemetry_imu_scaled(int16_t raw, int32_t full_scale) {
  int64_t value = (int64_t)raw * full_scale;
  return (int32_t)((value + (value < 0 ? -16384 : 16384)) / 32768);
}

inline bool telemetry_imu_saturated(int16_t raw) {
  return raw >= 32735 || raw <= -32735; // within 0.1% of either full-scale rail
}

inline bool telemetry_imu_bracket_valid(uint64_t lower, uint64_t upper, uint64_t read_end) {
  return lower && lower < upper && upper <= read_end;
}

#endif
