#include <initializer_list>
#include "../TelemetryImu.h"
#include <assert.h>
#include <limits.h>

struct FakeSpiImu {
  int calls = 0;
  int fail_at = 0;
  bool enableGyroscope() { return ++calls != fail_at; }
  bool enableAccelerometer() { return ++calls != fail_at; }
  int enableSyncSampleMode() { return ++calls != fail_at; }
};

int main() {
  for (int fail = 0; fail <= 3; ++fail) {
    FakeSpiImu sensor;
    sensor.fail_at = fail;
    int reads = 0;
    bool ok = telemetry_enable_spi_sync(sensor, [&]() { ++reads; return 0x83; });
    assert(ok == (fail == 0));
    assert(reads == (fail == 0 ? 1 : 0));
  }
  for (int control : {0x00, 0x03, 0x80, 0x81, 0x82}) {
    FakeSpiImu sensor;
    assert(!telemetry_enable_spi_sync(sensor, [&]() { return control; }));
  }

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
