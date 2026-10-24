#include "../TelemetryImu.h"
#include <assert.h>
#include <limits.h>

int main() {
  TelemetryImuContinuity state;
  assert(state.observe(0xfffffe) == 0);
  assert(state.observe(0xffffff) == 0);
  assert(state.observe(0) == 0); // a normal wrap is not loss/reset
  assert(state.pending_faults == 0);
  assert(state.observe(3) == 2);
  assert(state.pending_faults & TELEMETRY_IMU_SAMPLE_LOSS);
  state.delivery(false);
  assert(state.pending_faults & TELEMETRY_IMU_QUEUE_OVERFLOW);
  assert(state.observe(4) == 0);
  assert(state.pending_faults & TELEMETRY_IMU_SAMPLE_LOSS); // retained until delivered
  state.delivery(true);
  assert(state.pending_faults == 0);
  assert(state.observe(4) == 0);
  assert(state.pending_faults & TELEMETRY_IMU_TIMING_FAULT);
  state.delivery(true);
  assert(state.observe(1) == 0);
  assert(state.pending_faults & TELEMETRY_IMU_TIMING_FAULT);

  assert(!telemetry_imu_bracket_valid(0, 100, 110));
  assert(!telemetry_imu_bracket_valid(100, 100, 110));
  assert(!telemetry_imu_bracket_valid(100, 120, 110));
  assert(telemetry_imu_bracket_valid(100, 120, 130));
  assert(telemetry_imu_saturated(INT16_MIN));
  assert(telemetry_imu_saturated(INT16_MAX));
  assert(!telemetry_imu_saturated(32000));
  assert(telemetry_imu_scaled(INT16_MIN, 4000) == -4000);
  assert(telemetry_imu_scaled(8192, 4000) == 1000);
  assert(telemetry_imu_scaled(-8192, 512000) == -128000);
  assert(telemetry_imu_scaled(6400, 12800) == 2500); // 25 C = raw 25 * 256
  const uint8_t raw[] = {0x00, 0x80};
  assert(telemetry_imu_i16(raw) == INT16_MIN);
  return 0;
}
