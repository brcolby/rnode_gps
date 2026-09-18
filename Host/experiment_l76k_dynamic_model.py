#!/usr/bin/env python3
"""Verify a volatile L76K pedestrian-model change and restore the baseline."""

from __future__ import annotations

import argparse
import time

import serial

from rnode_broker import (
    CMD_TELEMETRY,
    GNSS_DYN_MODEL_SET,
    GNSS_NAVX_QUERY,
    KissStreamDecoder,
    NavxConfiguration,
    decode_telemetry,
    encode_kiss,
    encode_telemetry,
)


def transact(port: serial.Serial, subtype: int, body: bytes = b"", timeout: float = 4.0) -> NavxConfiguration:
    decoder = KissStreamDecoder()
    port.reset_input_buffer()
    port.write(encode_kiss(CMD_TELEMETRY, encode_telemetry(subtype, body)))
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for frame in decoder.feed(port.read(4096)):
            if frame.command != CMD_TELEMETRY or not frame.valid:
                continue
            try:
                message = decode_telemetry(frame.payload)
            except ValueError:
                continue
            if isinstance(message, NavxConfiguration):
                return message
    raise TimeoutError(f"no response for telemetry subtype 0x{subtype:02x}")


def query(port: serial.Serial) -> NavxConfiguration:
    result = transact(port, GNSS_NAVX_QUERY)
    if result.status != 0 or len(result.raw) != 44:
        raise RuntimeError(f"CFG-NAVX query failed with status {result.status}")
    return result


def set_model(port: serial.Serial, model: int) -> None:
    result = transact(port, GNSS_DYN_MODEL_SET, bytes([model]))
    if result.status != 0:
        raise RuntimeError(f"CFG-NAVX set failed with status {result.status}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.1, write_timeout=1.0) as port:
        baseline = query(port)
        original_model = baseline.raw[4]
        if original_model > 7:
            raise RuntimeError(f"refusing undocumented baseline dynamic model {original_model}")
        print(f"baseline dynamic model: {original_model}")
        try:
            set_model(port, 2)
            changed = query(port)
            print(f"pedestrian readback: {changed.raw[4]}")
            if changed.raw[4] != 2:
                raise RuntimeError("pedestrian model did not read back")
        finally:
            set_model(port, original_model)
            restored = query(port)
            print(f"restored dynamic model: {restored.raw[4]}")
            if restored.raw[4] != original_model:
                raise RuntimeError("baseline dynamic model was not restored")


if __name__ == "__main__":
    main()
