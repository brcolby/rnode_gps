#!/usr/bin/env python3
"""Query the attached telemetry firmware for the L76K CFG-NAVX state."""

from __future__ import annotations

import argparse
import json
import time

from rnode_broker import (
    CMD_TELEMETRY,
    GNSS_NAVX_QUERY,
    KissStreamDecoder,
    NavxConfiguration,
    decode_telemetry,
    encode_kiss,
    encode_telemetry,
)


STATUS_NAMES = {
    0: "ok",
    1: "timeout",
    2: "nack",
    3: "bad_frame",
    4: "busy",
}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=4.0)
    args = parser.parse_args()

    import serial

    decoder = KissStreamDecoder()
    with serial.Serial(args.port, args.baud, timeout=0.1, write_timeout=1.0) as port:
        port.write(encode_kiss(CMD_TELEMETRY, encode_telemetry(GNSS_NAVX_QUERY)))
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            for frame in decoder.feed(port.read(4096)):
                if frame.command != CMD_TELEMETRY or not frame.valid:
                    continue
                try:
                    message = decode_telemetry(frame.payload)
                except ValueError:
                    continue
                if isinstance(message, NavxConfiguration):
                    result = message.as_dict()
                    result["status_name"] = STATUS_NAMES.get(message.status, "unknown")
                    print(json.dumps(result, indent=2, sort_keys=True))
                    raise SystemExit(0 if message.status == 0 else 2)
    raise SystemExit("timed out waiting for firmware NAVX response")


if __name__ == "__main__":
    main()
