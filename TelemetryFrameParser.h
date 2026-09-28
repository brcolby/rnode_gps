// Copyright (C) 2026 rnode_gps contributors

#ifndef RNODE_GPS_TELEMETRY_FRAME_PARSER_H
#define RNODE_GPS_TELEMETRY_FRAME_PARSER_H

#include <stddef.h>
#include <stdint.h>

template <size_t Capacity>
struct TelemetryFrameBuffer {
  uint8_t data[Capacity];
  size_t length;
  bool escaped;
  bool malformed;

  void reset() {
    length = 0;
    escaped = false;
    malformed = false;
  }

  void push_encoded(uint8_t value) {
    if (value == 0xDB) {
      if (escaped) malformed = true;
      escaped = true;
      return;
    }
    if (escaped) {
      if (value == 0xDC) value = 0xC0;
      else if (value == 0xDD) value = 0xDB;
      else malformed = true;
      escaped = false;
    }
    if (length < Capacity) data[length++] = value;
    else malformed = true;
  }

  bool finish() {
    if (escaped) malformed = true;
    escaped = false;
    return !malformed;
  }
};

#endif
