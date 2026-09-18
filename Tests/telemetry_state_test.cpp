#include "../TelemetryFrameParser.h"
#include "../TelemetryState.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

size_t make_command(
  uint8_t subtype,
  const uint8_t *body,
  size_t body_length,
  uint8_t *payload,
  uint8_t version = TELEMETRY_PROTOCOL_VERSION
) {
  size_t offset = 0;
  payload[offset++] = version;
  payload[offset++] = subtype;
  for (size_t i = 0; i < body_length; i++) payload[offset++] = body[i];
  return telemetry_append_crc(payload, offset, TELEMETRY_MAX_PAYLOAD_BYTES);
}

void assert_error(const uint8_t *response, size_t length, uint8_t code) {
  assert(length == 5);
  assert(telemetry_valid_crc(response, length));
  assert(response[0] == TELEMETRY_PROTOCOL_VERSION);
  assert(response[1] == TELEMETRY_ERROR);
  assert(response[2] == code);
}

int main() {
  TelemetryState state = telemetry_initial_state();
  assert(state.ready == 0);
  assert(state.enabled == 0);
  assert(state.imu_rate_hz == 100);

  uint8_t request[TELEMETRY_MAX_PAYLOAD_BYTES] = {0};
  uint8_t response[TELEMETRY_MAX_PAYLOAD_BYTES] = {0};
  size_t request_length = make_command(TELEMETRY_CAPS_QUERY, NULL, 0, request);
  state.ready = TELEMETRY_GPS_READY | TELEMETRY_IMU_READY;
  size_t response_length = telemetry_process_command(
    state, request, request_length, false, 0x01, 0x56, response, sizeof(response)
  );
  const uint8_t expected_caps[] = {0x01, 0x01, 0x03, 0x03, 0x0f, 0x01, 0x56, 0x5c, 0x32};
  assert(response_length == sizeof(expected_caps));
  assert(memcmp(response, expected_caps, sizeof(expected_caps)) == 0);
  assert(state.enabled == 0); // capabilities never opt sensors in

  const uint8_t config_body[] = {0x03, 0x32};
  request_length = make_command(TELEMETRY_CONFIG_SET, config_body, sizeof(config_body), request);
  response_length = telemetry_process_command(
    state, request, request_length, false, 0x01, 0x56, response, sizeof(response)
  );
  const uint8_t expected_config[] = {0x01, 0x03, 0x03, 0x32, 0x03, 0x2a, 0x45};
  assert(response_length == sizeof(expected_config));
  assert(memcmp(response, expected_config, sizeof(expected_config)) == 0);
  assert(state.enabled == 0x03);
  assert(state.imu_rate_hz == 50);

  TelemetryState gps_only = telemetry_initial_state();
  gps_only.ready = TELEMETRY_GPS_READY;
  response_length = telemetry_process_command(
    gps_only, request, request_length, false, 0x01, 0x56, response, sizeof(response)
  );
  assert(response_length == 7);
  assert(response[2] == TELEMETRY_GPS_ENABLED);
  assert(gps_only.enabled == TELEMETRY_GPS_ENABLED);

  state.gps_sequence = 0xffff;
  state.imu_sequence = 0;
  state.invalid_nmea = 1;
  state.gps_drops = 2;
  state.imu_drops = 3;
  request_length = make_command(TELEMETRY_STATS_QUERY, NULL, 0, request);
  response_length = telemetry_process_command(
    state, request, request_length, false, 0x01, 0x56, response, sizeof(response)
  );
  const uint8_t expected_stats[] = {
    0x01, 0x05, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x13, 0x39,
  };
  assert(response_length == sizeof(expected_stats));
  assert(memcmp(response, expected_stats, sizeof(expected_stats)) == 0);

  assert(telemetry_take_sequence(state.gps_sequence) == 0xffff);
  assert(state.gps_sequence == 0);
  telemetry_record_send(true, state.gps_drops);
  assert(state.gps_drops == 2);
  telemetry_record_send(false, state.gps_drops);
  assert(state.gps_drops == 3);

  assert(telemetry_valid_nmea("$GPGGA,123*4A", 13));
  assert(!telemetry_valid_nmea("$GPGGA,123*00", 13));
  assert(!telemetry_valid_nmea("$GPGGA,123*4Aextra", 18));
  assert(telemetry_imu_due(20000, 0, 50));
  assert(!telemetry_imu_due(19999, 0, 50));
  assert(telemetry_imu_due(10, UINT64_MAX - 19989, 50));
  assert(!telemetry_imu_due(1000000, 0, 20));

  request[request_length - 1] ^= 0x01;
  response_length = telemetry_process_command(
    state, request, request_length, false, 0x01, 0x56, response, sizeof(response)
  );
  assert_error(response, response_length, TELEMETRY_ERROR_INTEGRITY);
  response_length = telemetry_process_command(
    state, request, request_length, true, 0x01, 0x56, response, sizeof(response)
  );
  assert_error(response, response_length, TELEMETRY_ERROR_INTEGRITY);

  request_length = make_command(TELEMETRY_CAPS_QUERY, NULL, 0, request, 0x02);
  response_length = telemetry_process_command(
    state, request, request_length, false, 0x01, 0x56, response, sizeof(response)
  );
  assert_error(response, response_length, TELEMETRY_ERROR_VERSION);

  request_length = make_command(0x06, NULL, 0, request);
  response_length = telemetry_process_command(
    state, request, request_length, false, 0x01, 0x56, response, sizeof(response)
  );
  assert_error(response, response_length, TELEMETRY_ERROR_SUBTYPE);

  const uint8_t bad_rate[] = {0x03, 20};
  request_length = make_command(TELEMETRY_CONFIG_SET, bad_rate, sizeof(bad_rate), request);
  response_length = telemetry_process_command(
    state, request, request_length, false, 0x01, 0x56, response, sizeof(response)
  );
  assert_error(response, response_length, TELEMETRY_ERROR_VALUE);

  TelemetryFrameBuffer<8> frame;
  frame.reset();
  const uint8_t escaped[] = {0x01, 0x20, 0xDB, 0xDC, 0xDB, 0xDD};
  for (uint8_t value : escaped) frame.push_encoded(value);
  assert(frame.finish());
  const uint8_t decoded[] = {0x01, 0x20, 0xC0, 0xDB};
  assert(frame.length == sizeof(decoded));
  assert(memcmp(frame.data, decoded, sizeof(decoded)) == 0);

  uint8_t radio_sentinel[] = {0xaa, 0xbb, 0xcc, 0xdd};
  frame.reset();
  frame.push_encoded(0xDB);
  frame.push_encoded(0x01);
  assert(!frame.finish());
  frame.reset();
  for (size_t i = 0; i < 20; i++) frame.push_encoded((uint8_t)i);
  assert(!frame.finish());
  const uint8_t expected_radio[] = {0xaa, 0xbb, 0xcc, 0xdd};
  assert(memcmp(radio_sentinel, expected_radio, sizeof(radio_sentinel)) == 0);

  frame.reset();
  frame.push_encoded(0x01);
  assert(frame.finish());
  assert(frame.length == 1 && frame.data[0] == 0x01);
  return 0;
}
