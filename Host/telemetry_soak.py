"""Deterministic PTY fault-injection soak for the telemetry broker."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import pty
import random
import select
import socket
import struct
import tempfile
import threading
import time
import tty
from pathlib import Path
from typing import Callable, Iterator, Sequence

try:
    from .rnode_broker import (
        CAPS_RESPONSE,
        CMD_RESET,
        CMD_TELEMETRY,
        CONFIG_STATE,
        FEND,
        FESC,
        GPS_NMEA,
        IMU_SAMPLE,
        RNodeBroker,
        KissFrame,
        KissStreamDecoder,
        encode_kiss,
        encode_telemetry,
    )
except ImportError:  # direct script execution
    from rnode_broker import (  # type: ignore[no-redef]
        CAPS_RESPONSE,
        CMD_RESET,
        CMD_TELEMETRY,
        CONFIG_STATE,
        FEND,
        FESC,
        GPS_NMEA,
        IMU_SAMPLE,
        RNodeBroker,
        KissFrame,
        KissStreamDecoder,
        encode_kiss,
        encode_telemetry,
    )


def _wait_for(predicate: Callable[[], bool], timeout_s: float = 3.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.002)
    raise TimeoutError("timed out waiting for broker state")


def _read_frame(
    fd: int,
    decoder: KissStreamDecoder,
    predicate: Callable[[KissFrame], bool],
    timeout_s: float = 3.0,
) -> KissFrame:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        readable, _, _ = select.select([fd], [], [], max(0.0, deadline - time.monotonic()))
        if not readable:
            break
        for frame in decoder.feed(os.read(fd, 65536)):
            if predicate(frame):
                return frame
    raise TimeoutError("timed out waiting for KISS frame")


def _write_fragmented(fd: int, data: bytes, rng: random.Random) -> None:
    offset = 0
    while offset < len(data):
        end = min(len(data), offset + rng.randint(1, 11))
        try:
            written = os.write(fd, data[offset:end])
        except BlockingIOError:
            select.select([], [fd], [], 1.0)
            continue
        offset += written


def _complete_lines(buffer: bytearray) -> Iterator[bytes]:
    while True:
        newline = buffer.find(b"\n")
        if newline < 0:
            return
        yield bytes(buffer[:newline]).rstrip(b"\r")
        del buffer[: newline + 1]


def _read_lines_fd(fd: int, buffer: bytearray, count: int, timeout_s: float = 3.0) -> list[bytes]:
    lines: list[bytes] = []
    deadline = time.monotonic() + timeout_s
    while len(lines) < count and time.monotonic() < deadline:
        lines.extend(_complete_lines(buffer))
        if len(lines) >= count:
            break
        readable, _, _ = select.select([fd], [], [], max(0.0, deadline - time.monotonic()))
        if readable:
            buffer.extend(os.read(fd, 65536))
    lines.extend(_complete_lines(buffer))
    if len(lines) < count:
        raise TimeoutError(f"received {len(lines)} of {count} expected GPS lines")
    return lines[:count]


def _read_lines_socket(
    channel: socket.socket, buffer: bytearray, count: int, timeout_s: float = 3.0
) -> list[bytes]:
    lines: list[bytes] = []
    deadline = time.monotonic() + timeout_s
    while len(lines) < count and time.monotonic() < deadline:
        lines.extend(_complete_lines(buffer))
        if len(lines) >= count:
            break
        readable, _, _ = select.select(
            [channel], [], [], max(0.0, deadline - time.monotonic())
        )
        if readable:
            chunk = channel.recv(65536)
            if not chunk:
                raise ConnectionError("IMU client disconnected before pending records drained")
            buffer.extend(chunk)
    lines.extend(_complete_lines(buffer))
    if len(lines) < count:
        raise TimeoutError(f"received {len(lines)} of {count} expected IMU lines")
    return lines[:count]


def _nmea(index: int) -> bytes:
    body = f"GPGGA,{index:06d}"
    checksum = 0
    for value in body.encode("ascii"):
        checksum ^= value
    return f"${body}*{checksum:02X}".encode("ascii")


def _sha256(chunks: list[bytes]) -> str:
    digest = hashlib.sha256()
    for chunk in chunks:
        digest.update(chunk)
    return digest.hexdigest()


def run_soak(
    *, cycles: int = 256, seed: int = 0x475354, reset_interval: int = 83, slow_interval: int = 13
) -> dict:
    if cycles < 16:
        raise ValueError("cycles must be at least 16")
    if reset_interval < 8:
        raise ValueError("reset_interval must be at least 8")
    if slow_interval < 2:
        raise ValueError("slow_interval must be at least 2")

    rng = random.Random(seed)
    physical_master, physical_slave = pty.openpty()
    tty.setraw(physical_slave)
    physical_path = os.ttyname(physical_slave)
    os.close(physical_slave)
    os.set_blocking(physical_master, False)

    stop = threading.Event()
    errors: list[BaseException] = []
    broker_thread: threading.Thread | None = None
    rnode_fd = gps_fd = -1
    imu_client: socket.socket | None = None
    device_decoder = KissStreamDecoder()
    rnode_decoder = KissStreamDecoder()
    gps_buffer = bytearray()
    imu_buffer = bytearray()

    expected_device_radio: list[bytes] = []
    actual_device_radio: list[bytes] = []
    expected_host_radio: list[bytes] = []
    actual_host_radio: list[bytes] = []
    expected_gps: list[bytes] = []
    actual_gps: list[bytes] = []
    expected_imu_sequences: list[int] = []
    actual_imu_sequences: list[int] = []
    pending_gps = pending_imu = 0
    crc_faults = escape_faults = resets = rollovers = 0
    recovery_ms: list[float] = []
    max_physical_queue = max_rnode_queue = max_gps_queue = 0
    max_pending_gps = max_pending_imu = 0
    endpoint_cleanup = False
    thread_stopped = False

    with tempfile.TemporaryDirectory(prefix="rnode-gps-soak-", dir="/private/tmp") as directory:
        runtime = Path(directory)
        broker = RNodeBroker(
            port=physical_path,
            rnode_link=runtime / "rnode",
            gps_link=runtime / "gps",
            imu_socket=runtime / "imu.sock",
            settle_time_s=0.0,
        )

        def broker_target() -> None:
            try:
                broker.run(stop)
            except BaseException as exc:
                errors.append(exc)

        def handshake() -> None:
            query = _read_frame(
                physical_master,
                device_decoder,
                lambda frame: frame.command == CMD_TELEMETRY
                and frame.payload == encode_telemetry(0x00),
            )
            if not query.valid:
                raise AssertionError("broker emitted malformed capabilities query")
            capabilities = encode_telemetry(
                CAPS_RESPONSE, bytes((0x03, 0x03, 0x0F, 0x01, 0x56))
            )
            _write_fragmented(
                physical_master, encode_kiss(CMD_TELEMETRY, capabilities), rng
            )
            configure = _read_frame(
                physical_master,
                device_decoder,
                lambda frame: frame.command == CMD_TELEMETRY
                and frame.payload == encode_telemetry(0x02, bytes((0x03, 50))),
            )
            if not configure.valid:
                raise AssertionError("broker emitted malformed configuration command")
            configured = encode_telemetry(CONFIG_STATE, bytes((0x03, 50, 0x03)))
            _write_fragmented(
                physical_master, encode_kiss(CMD_TELEMETRY, configured), rng
            )
            _wait_for(lambda: broker.configuration is not None or bool(errors))
            if errors:
                raise errors[0]

        def connect_imu() -> socket.socket:
            channel = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            channel.settimeout(3.0)
            channel.connect(str(runtime / "imu.sock"))
            _wait_for(lambda: len(broker.imu_clients) == 1 or bool(errors))
            if errors:
                raise errors[0]
            return channel

        def drain_sensors() -> None:
            nonlocal pending_gps, pending_imu
            if pending_gps:
                actual_gps.extend(_read_lines_fd(gps_fd, gps_buffer, pending_gps))
                pending_gps = 0
            if pending_imu:
                assert imu_client is not None
                records = _read_lines_socket(imu_client, imu_buffer, pending_imu)
                for record in records:
                    actual_imu_sequences.append(int(json.loads(record)["sequence"]))
                pending_imu = 0

        broker_thread = threading.Thread(target=broker_target, daemon=True)
        broker_thread.start()

        try:
            _wait_for(lambda: (runtime / "rnode").exists() or bool(errors))
            if errors:
                raise errors[0]
            handshake()
            rnode_fd = os.open(runtime / "rnode", os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            gps_fd = os.open(runtime / "gps", os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            imu_client = connect_imu()

            gps_sequence = 0xFFF0
            imu_sequence = 0xFFF0
            for cycle in range(cycles):
                if cycle and cycle % reset_interval == 0:
                    drain_sensors()
                    reset = encode_kiss(CMD_RESET, b"\xf8")
                    expected_device_radio.append(reset)
                    began = time.monotonic()
                    _write_fragmented(physical_master, reset, rng)
                    forwarded = _read_frame(
                        rnode_fd,
                        rnode_decoder,
                        lambda frame: frame.command == CMD_RESET,
                    )
                    actual_device_radio.append(forwarded.raw)
                    _wait_for(lambda: broker.configuration is None or bool(errors))
                    handshake()
                    recovery_ms.append((time.monotonic() - began) * 1000.0)
                    resets += 1
                    assert imu_client is not None
                    _wait_for(lambda: len(broker.imu_clients) == 0 or bool(errors))
                    imu_client.close()
                    imu_client = connect_imu()
                    imu_buffer.clear()
                    gps_sequence = 0
                    imu_sequence = 0

                if cycle % 29 == 7:
                    corrupt = bytearray(
                        encode_telemetry(0x05, struct.pack(">HHIII", 1, 2, 3, 4, 5))
                    )
                    corrupt[-1] ^= 0x01
                    _write_fragmented(
                        physical_master, encode_kiss(CMD_TELEMETRY, corrupt), rng
                    )
                    crc_faults += 1
                if cycle % 31 == 9:
                    _write_fragmented(
                        physical_master,
                        bytes((FEND, CMD_TELEMETRY, FESC, 0x01, FEND)),
                        rng,
                    )
                    escape_faults += 1

                device_radio = encode_kiss(
                    0x23,
                    struct.pack(">I", cycle) + bytes((FEND, FESC, cycle & 0xFF)),
                )
                host_radio = encode_kiss(
                    0x13,
                    struct.pack(">I", cycle) + bytes((FESC, FEND, (cycle * 7) & 0xFF)),
                )
                nmea = _nmea(cycle)
                gps_payload = encode_telemetry(
                    GPS_NMEA, struct.pack(">HQ", gps_sequence, cycle * 100_000) + nmea
                )
                imu_body = struct.pack(
                    ">HQhhhiiihB",
                    imu_sequence,
                    cycle * 100_000 + 1,
                    cycle % 100,
                    -(cycle % 100),
                    1000,
                    cycle,
                    -cycle,
                    cycle * 2,
                    2500,
                    3,
                )
                imu_payload = encode_telemetry(IMU_SAMPLE, imu_body)

                combined = (
                    device_radio
                    + encode_kiss(CMD_TELEMETRY, gps_payload)
                    + encode_kiss(CMD_TELEMETRY, imu_payload)
                )
                _write_fragmented(physical_master, combined, rng)
                expected_device_radio.append(device_radio)
                expected_gps.append(nmea)
                expected_imu_sequences.append(imu_sequence)
                pending_gps += 1
                pending_imu += 1
                max_pending_gps = max(max_pending_gps, pending_gps)
                max_pending_imu = max(max_pending_imu, pending_imu)

                forwarded = _read_frame(
                    rnode_fd,
                    rnode_decoder,
                    lambda frame: frame.command == 0x23,
                )
                actual_device_radio.append(forwarded.raw)

                _write_fragmented(rnode_fd, host_radio, rng)
                expected_host_radio.append(host_radio)
                outbound = _read_frame(
                    physical_master,
                    device_decoder,
                    lambda frame: frame.command == 0x13,
                )
                actual_host_radio.append(outbound.raw)

                if gps_sequence == 0xFFFF or imu_sequence == 0xFFFF:
                    rollovers += 1
                gps_sequence = (gps_sequence + 1) & 0xFFFF
                imu_sequence = (imu_sequence + 1) & 0xFFFF

                max_physical_queue = max(max_physical_queue, len(broker.physical_output))
                max_rnode_queue = max(max_rnode_queue, len(broker.rnode_output))
                max_gps_queue = max(max_gps_queue, broker.gps_output_bytes)

                # Deliberately lag both sensor consumers for bounded bursts.
                if (cycle + 1) % slow_interval == 0:
                    drain_sensors()

            drain_sensors()
        finally:
            stop.set()
            if broker_thread is not None:
                broker_thread.join(3.0)
                thread_stopped = not broker_thread.is_alive()
            if imu_client is not None:
                imu_client.close()
            if rnode_fd >= 0:
                os.close(rnode_fd)
            if gps_fd >= 0:
                os.close(gps_fd)
            os.close(physical_master)
            endpoint_cleanup = not any(
                os.path.lexists(runtime / name) for name in ("rnode", "gps", "imu.sock")
            )

    device_exact = expected_device_radio == actual_device_radio
    host_exact = expected_host_radio == actual_host_radio
    gps_exact = expected_gps == actual_gps
    # Exact sequence equality is authoritative across reset epochs; a count
    # delta is retained as a compact diagnostic if equality ever fails.
    imu_exact = expected_imu_sequences == actual_imu_sequences
    imu_gaps = 0 if imu_exact else max(0, len(expected_imu_sequences) - len(actual_imu_sequences))
    passed = all(
        (
            not errors,
            device_exact,
            host_exact,
            gps_exact,
            imu_exact,
            thread_stopped,
            endpoint_cleanup,
            max_physical_queue < 1024 * 1024,
            max_rnode_queue < 1024 * 1024,
            max_gps_queue <= 64 * 1024,
            not recovery_ms or max(recovery_ms) < 2000.0,
        )
    )
    return {
        "ok": passed,
        "parameters": {
            "cycles": cycles,
            "seed": seed,
            "reset_interval": reset_interval,
            "slow_interval": slow_interval,
        },
        "radio": {
            "device_to_host": {
                "expected_frames": len(expected_device_radio),
                "actual_frames": len(actual_device_radio),
                "expected_bytes": sum(map(len, expected_device_radio)),
                "actual_bytes": sum(map(len, actual_device_radio)),
                "expected_sha256": _sha256(expected_device_radio),
                "actual_sha256": _sha256(actual_device_radio),
                "exact": device_exact,
            },
            "host_to_device": {
                "expected_frames": len(expected_host_radio),
                "actual_frames": len(actual_host_radio),
                "expected_bytes": sum(map(len, expected_host_radio)),
                "actual_bytes": sum(map(len, actual_host_radio)),
                "expected_sha256": _sha256(expected_host_radio),
                "actual_sha256": _sha256(actual_host_radio),
                "exact": host_exact,
            },
        },
        "gps": {
            "expected": len(expected_gps),
            "received": len(actual_gps),
            "drops": len(expected_gps) - len(actual_gps),
            "exact": gps_exact,
        },
        "imu": {
            "expected": len(expected_imu_sequences),
            "received": len(actual_imu_sequences),
            "drops": len(expected_imu_sequences) - len(actual_imu_sequences),
            "aggregate_gap_diagnostic": imu_gaps,
            "exact": imu_exact,
        },
        "faults": {
            "crc": crc_faults,
            "invalid_escape": escape_faults,
            "resets": resets,
            "sequence_rollovers": rollovers,
            "recovery_ms": recovery_ms,
            "max_recovery_ms": max(recovery_ms, default=0.0),
        },
        "queue_bounds": {
            "max_physical_bytes": max_physical_queue,
            "physical_limit": 1024 * 1024,
            "max_rnode_bytes": max_rnode_queue,
            "rnode_limit": 1024 * 1024,
            "max_gps_bytes": max_gps_queue,
            "gps_limit": 64 * 1024,
            "max_delayed_gps_records": max_pending_gps,
            "max_delayed_imu_records": max_pending_imu,
        },
        "cleanup": {
            "broker_thread_stopped": thread_stopped,
            "runtime_endpoints_removed": endpoint_cleanup,
        },
        "broker_errors": [repr(error) for error in errors],
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cycles", type=int, default=256)
    parser.add_argument("--seed", type=lambda value: int(value, 0), default=0x475354)
    parser.add_argument("--reset-interval", type=int, default=83)
    parser.add_argument("--slow-interval", type=int, default=13)
    parser.add_argument("--output", type=Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        result = run_soak(
            cycles=args.cycles,
            seed=args.seed,
            reset_interval=args.reset_interval,
            slow_interval=args.slow_interval,
        )
    except Exception as exc:
        result = {"ok": False, "error": repr(exc)}
    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered)
    print(rendered, end="")
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
