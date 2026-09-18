#!/usr/bin/env python3
"""Temporarily set the L76K CFG-NAVX dynamic model (not saved to flash)."""

from __future__ import annotations

import argparse
import json
import time

from rnode_broker import (
    CMD_TELEMETRY,
    GNSS_DYN_MODEL_SET,
    KissStreamDecoder,
    NavxConfiguration,
    decode_telemetry,
    encode_kiss,
    encode_telemetry,
)


MODEL_NAMES = {
    "portable": 0,
    "stationary": 1,
    "pedestrian": 2,
    "automotive": 3,
    "marine": 4,
    "airborne1g": 5,
    "airborne2g": 6,
    "airborne4g": 7,
}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port")
    parser.add_argument("model", choices=MODEL_NAMES)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=4.0)
    args = parser.parse_args()

    import serial

    decoder = KissStreamDecoder()
    request = encode_telemetry(GNSS_DYN_MODEL_SET, bytes([MODEL_NAMES[args.model]]))
    with serial.Serial(args.port, args.baud, timeout=0.1, write_timeout=1.0) as port:
        port.write(encode_kiss(CMD_TELEMETRY, request))
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            for frame in decoder.feed(port.read(4096)):
                if frame.command != CMD_TELEMETRY or not frame.valid:
                    continue
                try:
                    message = decode_telemetry(frame.payload)
                except ValueError:
                    continue
                if isinstance(message, NavxConfiguration) and not message.raw:
                    print(json.dumps({"model": args.model, "status": message.status}))
                    raise SystemExit(0 if message.status == 0 else 2)
    raise SystemExit("timed out waiting for firmware dynamic-model response")


if __name__ == "__main__":
    main()
