#include "../TelemetryProtocol.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

struct Vector {
  const uint8_t *content;
  size_t content_length;
  uint16_t crc;
};

int main() {
  const uint8_t caps_query[] = {0x01, 0x00};
  const uint8_t caps_response[] = {0x01, 0x01, 0x03, 0x03, 0x0f, 0x01, 0x56};
  const uint8_t configure[] = {0x01, 0x02, 0x03, 0x32};
  const uint8_t configuration[] = {0x01, 0x03, 0x03, 0x32, 0x03};
  const uint8_t stats_query[] = {0x01, 0x04};
  const uint8_t stats_response[] = {
    0x01, 0x05, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03,
  };
  const uint8_t gps[] = {
    0x01, 0x10, 0xff, 0xff, 0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
    0x24, 0x47, 0x50, 0x47, 0x47, 0x41, 0x2c, 0x31, 0x32, 0x33, 0x2a, 0x34, 0x41,
  };
  const uint8_t imu[] = {
    0x01, 0x20, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x80, 0x00, 0x00, 0x00, 0x7f, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0x80, 0x00, 0xc0,
  };
  const uint8_t error[] = {0x01, 0x7f, 0xc0};
  const Vector vectors[] = {
    {caps_query, sizeof(caps_query), 0x2e3e},
    {caps_response, sizeof(caps_response), 0x5c32},
    {configure, sizeof(configure), 0xdf56},
    {configuration, sizeof(configuration), 0x2a45},
    {stats_query, sizeof(stats_query), 0x6eba},
    {stats_response, sizeof(stats_response), 0x1339},
    {gps, sizeof(gps), 0xd3dd},
    {imu, sizeof(imu), 0x88dc},
    {error, sizeof(error), 0x3a87},
  };

  const uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  assert(telemetry_crc16(check, sizeof(check)) == 0x29b1);

  for (const Vector &vector : vectors) {
    assert(telemetry_crc16(vector.content, vector.content_length) == vector.crc);
    uint8_t payload[TELEMETRY_MAX_PAYLOAD_BYTES] = {0};
    memcpy(payload, vector.content, vector.content_length);
    size_t length = telemetry_append_crc(payload, vector.content_length, sizeof(payload));
    assert(length == vector.content_length + TELEMETRY_CRC_BYTES);
    assert(payload[length - 2] == (uint8_t)(vector.crc >> 8));
    assert(payload[length - 1] == (uint8_t)vector.crc);
    assert(telemetry_valid_crc(payload, length));
    payload[0] ^= 0x01;
    assert(!telemetry_valid_crc(payload, length));
  }

  uint8_t full[TELEMETRY_MAX_PAYLOAD_BYTES] = {0};
  assert(telemetry_append_crc(full, sizeof(full) - TELEMETRY_CRC_BYTES, sizeof(full)) == sizeof(full));
  assert(telemetry_append_crc(full, sizeof(full) - 1, sizeof(full)) == 0);
  return 0;
}
