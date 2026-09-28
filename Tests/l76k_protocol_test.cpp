#include "../L76KProtocol.h"

#include <assert.h>
#include <stdint.h>

int main() {
  uint8_t set_frame[54];
  assert(l76k_build_navx_dynamic_model_set(2, set_frame, sizeof(set_frame)) == sizeof(set_frame));
  assert(set_frame[0] == 0xBA && set_frame[1] == 0xCE);
  assert(set_frame[2] == 44 && set_frame[4] == 0x06 && set_frame[5] == 0x07);
  assert(set_frame[6] == 1 && set_frame[10] == 2);
  assert(l76k_build_navx_dynamic_model_set(8, set_frame, sizeof(set_frame)) == 0);

  L76KCasicParser set_parser;
  L76KCasicEvent set_event = L76K_CASIC_NONE;
  for (size_t i = 0; i < sizeof(set_frame); ++i) {
    assert(set_parser.consume(set_frame[i], set_event));
  }
  assert(set_event == L76K_CASIC_FRAME);
  assert(set_parser.message_class == L76K_CASIC_CLASS_CFG);
  assert(set_parser.message_id == L76K_CASIC_ID_NAVX);
  assert(set_parser.length == L76K_NAVX_PAYLOAD_BYTES);
  assert(set_parser.payload[0] == 1);
  assert(set_parser.payload[4] == 2);

  // A synthetic 44-byte CFG-NAVX response with a valid CASIC checksum.
  uint8_t frame[54] = {0xBA, 0xCE, 44, 0, 0x06, 0x07};
  for (uint8_t i = 0; i < 44; i++) frame[6 + i] = i;
  uint32_t checksum = 44 | (uint32_t)0x06 << 16 | (uint32_t)0x07 << 24;
  for (uint8_t i = 0; i < 44; i += 4) {
    checksum += (uint32_t)frame[6 + i] |
                (uint32_t)frame[7 + i] << 8 |
                (uint32_t)frame[8 + i] << 16 |
                (uint32_t)frame[9 + i] << 24;
  }
  frame[50] = checksum;
  frame[51] = checksum >> 8;
  frame[52] = checksum >> 16;
  frame[53] = checksum >> 24;

  L76KCasicParser parser;
  L76KCasicEvent event = L76K_CASIC_NONE;
  assert(!parser.consume('$', event));
  for (size_t i = 0; i < sizeof(frame); i++) {
    assert(parser.consume(frame[i], event));
    if (i + 1 < sizeof(frame)) assert(event == L76K_CASIC_NONE);
  }
  assert(event == L76K_CASIC_FRAME);
  assert(parser.message_class == L76K_CASIC_CLASS_CFG);
  assert(parser.message_id == L76K_CASIC_ID_NAVX);
  assert(parser.length == L76K_NAVX_PAYLOAD_BYTES);
  assert(parser.payload[0] == 0 && parser.payload[43] == 43);

  frame[20] ^= 1;
  for (size_t i = 0; i < sizeof(frame); i++) parser.consume(frame[i], event);
  assert(event == L76K_CASIC_BAD_FRAME);
  return 0;
}
