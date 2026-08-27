"""Single-port RNode, GNSS and IMU broker for the T-Beam Supreme fork."""

from __future__ import annotations

import argparse
import errno
import json
import logging
import math
import os
import pty
import selectors
import socket
import stat
import struct
import sys
import threading
import time
import tty
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, Sequence, Union


FEND = 0xC0
FESC = 0xDB
TFEND = 0xDC
TFESC = 0xDD
CMD_RESET = 0x55
CMD_TELEMETRY = 0xA0

PROTOCOL_VERSION = 0x01
CAPS_QUERY = 0x00
CAPS_RESPONSE = 0x01
CONFIG_SET = 0x02
CONFIG_STATE = 0x03
STATS_QUERY = 0x04
STATS_RESPONSE = 0x05
GPS_NMEA = 0x10
IMU_SAMPLE = 0x20
ERROR = 0x7F

GPS_ENABLED = 0x01
IMU_ENABLED = 0x02
SUPPORTED_IMU_RATES = (10, 25, 50, 100)

LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class KissFrame:
    """A decoded KISS frame plus its exact bytes for transparent forwarding."""

    command: int | None
    payload: bytes
    raw: bytes


class KissStreamDecoder:
    """Incrementally split KISS without rewriting frames that are forwarded."""

    def __init__(self, *, max_frame_size: int = 8192) -> None:
        self.max_frame_size = max_frame_size
        self._in_frame = False
        self._escaped = False
        self._raw = bytearray()
        self._decoded = bytearray()

    def reset(self) -> None:
        self._in_frame = False
        self._escaped = False
        self._raw.clear()
        self._decoded.clear()

    def feed(self, data: bytes) -> Iterator[KissFrame]:
        for value in data:
            if value == FEND:
                completed = None
                if self._in_frame and self._decoded:
                    raw = bytes(self._raw) + bytes((FEND,))
                    completed = KissFrame(self._decoded[0], bytes(self._decoded[1:]), raw)
                self._in_frame = True
                self._escaped = False
                self._raw = bytearray((FEND,))
                self._decoded.clear()
                if completed is not None:
                    yield completed
                continue

            if not self._in_frame:
                # Preserve diagnostics or boot noise that precedes the first
                # KISS delimiter. It is never interpreted as telemetry.
                yield KissFrame(None, b"", bytes((value,)))
                continue

            self._raw.append(value)
            if len(self._raw) > self.max_frame_size:
                raw = bytes(self._raw)
                self.reset()
                yield KissFrame(None, b"", raw)
                continue

            if value == FESC:
                self._escaped = True
            elif self._escaped:
                if value == TFEND:
                    self._decoded.append(FEND)
                elif value == TFESC:
                    self._decoded.append(FESC)
                else:
                    self._decoded.append(value)
                self._escaped = False
            else:
                self._decoded.append(value)


def encode_kiss(command: int, payload: bytes = b"") -> bytes:
    """Encode one KISS frame."""

    encoded = bytearray((FEND, command))
    for value in payload:
        if value == FEND:
            encoded.extend((FESC, TFEND))
        elif value == FESC:
            encoded.extend((FESC, TFESC))
        else:
            encoded.append(value)
    encoded.append(FEND)
    return bytes(encoded)


@dataclass(frozen=True)
class Capabilities:
    supported: int
    ready: int
    rate_mask: int
    firmware_major: int
    firmware_minor: int


@dataclass(frozen=True)
class Configuration:
    enabled: int
    imu_rate_hz: int
    ready: int


@dataclass(frozen=True)
class GPSMessage:
    sequence: int
    device_time_us: int
    sentence: str


@dataclass(frozen=True)
class IMUMessage:
    sequence: int
    device_time_us: int
    accel_mg: tuple[int, int, int]
    gyro_mdps: tuple[int, int, int]
    temperature_centi_c: int
    status: int

    def as_json(self) -> bytes:
        gravity = 9.80665 / 1000.0
        radians = math.pi / (180.0 * 1000.0)
        payload = {
            "sequence": self.sequence,
            "device_time_us": self.device_time_us,
            "host_time_ns": time.time_ns(),
            "accel_mg": dict(zip(("x", "y", "z"), self.accel_mg)),
            "accel_m_s2": dict(
                zip(("x", "y", "z"), (value * gravity for value in self.accel_mg))
            ),
            "gyro_mdps": dict(zip(("x", "y", "z"), self.gyro_mdps)),
            "gyro_rad_s": dict(
                zip(("x", "y", "z"), (value * radians for value in self.gyro_mdps))
            ),
            "temperature_c": self.temperature_centi_c / 100.0,
            "status": self.status,
        }
        return (json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8")


@dataclass(frozen=True)
class Statistics:
    gps_sequence: int
    imu_sequence: int
    invalid_nmea: int
    gps_drops: int
    imu_drops: int


@dataclass(frozen=True)
class TelemetryError:
    code: int


TelemetryMessage = Union[
    Capabilities,
    Configuration,
    GPSMessage,
    IMUMessage,
    Statistics,
    TelemetryError,
]


def decode_telemetry(payload: bytes) -> TelemetryMessage:
    """Decode a command-0xA0 payload from the firmware extension."""

    if len(payload) < 2:
        raise ValueError("telemetry payload is too short")
    if payload[0] != PROTOCOL_VERSION:
        raise ValueError(f"unsupported telemetry protocol version {payload[0]}")

    subtype = payload[1]
    body = payload[2:]
    if subtype == CAPS_RESPONSE and len(body) == 5:
        return Capabilities(*body)
    if subtype == CONFIG_STATE and len(body) == 3:
        return Configuration(*body)
    if subtype == GPS_NMEA and len(body) >= 10:
        sequence, device_time_us = struct.unpack(">HQ", body[:10])
        return GPSMessage(sequence, device_time_us, body[10:].decode("ascii"))
    if subtype == IMU_SAMPLE and len(body) == 31:
        values = struct.unpack(">HQhhhiiihB", body)
        return IMUMessage(
            sequence=values[0],
            device_time_us=values[1],
            accel_mg=(values[2], values[3], values[4]),
            gyro_mdps=(values[5], values[6], values[7]),
            temperature_centi_c=values[8],
            status=values[9],
        )
    if subtype == STATS_RESPONSE and len(body) == 16:
        return Statistics(*struct.unpack(">HHIII", body))
    if subtype == ERROR and len(body) == 1:
        return TelemetryError(body[0])
    raise ValueError(f"invalid telemetry subtype/length: 0x{subtype:02x}/{len(body)}")


def _make_pty(link: Path) -> tuple[int, int, str]:
    link.parent.mkdir(parents=True, exist_ok=True)
    master_fd, slave_fd = pty.openpty()
    tty.setraw(slave_fd)
    os.set_blocking(master_fd, False)
    target = os.ttyname(slave_fd)

    if os.path.lexists(link):
        if not link.is_symlink():
            os.close(master_fd)
            os.close(slave_fd)
            raise FileExistsError(f"refusing to replace non-symlink endpoint: {link}")
        if link.exists():
            os.close(master_fd)
            os.close(slave_fd)
            raise FileExistsError(f"serial endpoint is already active: {link}")
        link.unlink()
    link.symlink_to(target)
    return master_fd, slave_fd, target


def _make_unix_server(path: Path) -> socket.socket:
    path.parent.mkdir(parents=True, exist_ok=True)
    if os.path.lexists(path):
        mode = path.lstat().st_mode
        if not stat.S_ISSOCK(mode):
            raise FileExistsError(f"refusing to replace non-socket endpoint: {path}")
        probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            probe.connect(str(path))
        except ConnectionRefusedError:
            path.unlink()
        else:
            raise FileExistsError(f"IMU endpoint is already active: {path}")
        finally:
            probe.close()
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.setblocking(False)
    server.bind(str(path))
    server.listen(8)
    return server


class RNodeBroker:
    """Own the real serial port and expose stock-RNode, GPS and IMU endpoints."""

    def __init__(
        self,
        *,
        port: str,
        baudrate: int = 115200,
        rnode_link: Path = Path("/run/rnode-gps/rnode"),
        gps_link: Path = Path("/run/rnode-gps/gps"),
        imu_socket: Path = Path("/run/rnode-gps/imu.sock"),
        enable_gps: bool = True,
        enable_imu: bool = True,
        imu_rate_hz: int = 50,
        settle_time_s: float = 1.0,
    ) -> None:
        if imu_rate_hz not in SUPPORTED_IMU_RATES:
            raise ValueError(f"unsupported IMU rate {imu_rate_hz}")
        self.port = port
        self.baudrate = baudrate
        self.rnode_link = rnode_link
        self.gps_link = gps_link
        self.imu_socket_path = imu_socket
        self.enable_flags = (GPS_ENABLED if enable_gps else 0) | (IMU_ENABLED if enable_imu else 0)
        self.imu_rate_hz = imu_rate_hz
        self.settle_time_s = settle_time_s

        self.selector = selectors.DefaultSelector()
        self.physical_decoder = KissStreamDecoder()
        self.host_decoder = KissStreamDecoder()
        self.serial_port = None
        self.rnode_master = -1
        self.rnode_slave = -1
        self.rnode_target: str | None = None
        self.gps_master = -1
        self.gps_slave = -1
        self.gps_target: str | None = None
        self.imu_server: socket.socket | None = None
        self.imu_clients: set[socket.socket] = set()
        self.rnode_output = bytearray()
        self.gps_output = bytearray()
        self.capabilities: Capabilities | None = None
        self.configuration: Configuration | None = None
        self.last_gps_sequence: int | None = None
        self.last_imu_sequence: int | None = None
        self.next_caps_query = 0.0

    def _open(self) -> None:
        try:
            import serial  # type: ignore[import-not-found]
        except ImportError as exc:
            raise RuntimeError("rnode_broker.py requires pyserial: pip install -r Host/requirements.txt") from exc

        self.serial_port = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=0,
            write_timeout=1.0,
            rtscts=False,
            dsrdtr=False,
        )
        self.rnode_master, self.rnode_slave, self.rnode_target = _make_pty(self.rnode_link)
        self.gps_master, self.gps_slave, self.gps_target = _make_pty(self.gps_link)
        self.imu_server = _make_unix_server(self.imu_socket_path)

        self.selector.register(self.serial_port.fileno(), selectors.EVENT_READ, "physical")
        self.selector.register(self.rnode_master, selectors.EVENT_READ, "rnode")
        self.selector.register(self.imu_server, selectors.EVENT_READ, "imu_server")
        self.next_caps_query = time.monotonic() + self.settle_time_s
        LOGGER.info("physical RNode: %s at %d baud", self.port, self.baudrate)
        LOGGER.info("RNode PTY: %s -> %s", self.rnode_link, self.rnode_target)
        LOGGER.info("GPS PTY: %s -> %s", self.gps_link, self.gps_target)
        LOGGER.info("IMU socket: %s", self.imu_socket_path)

    def _send_physical(self, frame: bytes) -> None:
        assert self.serial_port is not None
        written = self.serial_port.write(frame)
        if written != len(frame):
            raise OSError(f"short serial write: {written}/{len(frame)}")

    def _query_capabilities(self) -> None:
        self._send_physical(encode_kiss(CMD_TELEMETRY, bytes((PROTOCOL_VERSION, CAPS_QUERY))))
        self.next_caps_query = time.monotonic() + 2.0

    def _configure(self, capabilities: Capabilities) -> None:
        flags = self.enable_flags & capabilities.supported & capabilities.ready
        payload = bytes((PROTOCOL_VERSION, CONFIG_SET, flags, self.imu_rate_hz))
        self._send_physical(encode_kiss(CMD_TELEMETRY, payload))

    def _queue_rnode(self, data: bytes) -> None:
        if len(self.rnode_output) + len(data) > 1024 * 1024:
            raise BufferError("RNode PTY consumer is more than 1 MiB behind")
        self.rnode_output.extend(data)

    def _queue_gps(self, sentence: str) -> None:
        data = sentence.encode("ascii") + b"\r\n"
        if len(self.gps_output) + len(data) > 64 * 1024:
            LOGGER.warning("GPS PTY consumer is behind; dropping buffered NMEA")
            self.gps_output.clear()
        self.gps_output.extend(data)

    @staticmethod
    def _flush_fd(fd: int, pending: bytearray) -> None:
        if not pending:
            return
        try:
            written = os.write(fd, pending)
        except BlockingIOError:
            return
        except OSError as exc:
            if exc.errno in (errno.EIO, errno.ENXIO):
                return
            raise
        del pending[:written]

    def _broadcast_imu(self, sample: IMUMessage) -> None:
        line = sample.as_json()
        for client in tuple(self.imu_clients):
            try:
                written = client.send(line)
                if written != len(line):
                    raise BlockingIOError
            except (BlockingIOError, BrokenPipeError, ConnectionResetError):
                self.imu_clients.discard(client)
                client.close()

    @staticmethod
    def _warn_sequence_gap(stream: str, previous: int | None, current: int) -> None:
        if previous is None:
            return
        expected = (previous + 1) & 0xFFFF
        if current != expected:
            dropped = (current - expected) & 0xFFFF
            LOGGER.warning("%s telemetry sequence gap: expected %d, got %d (%d lost)", stream, expected, current, dropped)

    def _handle_telemetry(self, payload: bytes) -> None:
        try:
            message = decode_telemetry(payload)
        except (UnicodeDecodeError, ValueError) as exc:
            LOGGER.warning("discarding invalid telemetry frame: %s", exc)
            return

        if isinstance(message, Capabilities):
            self.capabilities = message
            LOGGER.info(
                "telemetry ready (firmware %d.%d, supported=0x%02x, ready=0x%02x)",
                message.firmware_major,
                message.firmware_minor,
                message.supported,
                message.ready,
            )
            self._configure(message)
        elif isinstance(message, Configuration):
            self.configuration = message
            self.next_caps_query = math.inf
            LOGGER.info("telemetry configured: flags=0x%02x, IMU=%d Hz", message.enabled, message.imu_rate_hz)
        elif isinstance(message, GPSMessage):
            self._warn_sequence_gap("GPS", self.last_gps_sequence, message.sequence)
            self.last_gps_sequence = message.sequence
            self._queue_gps(message.sentence)
        elif isinstance(message, IMUMessage):
            self._warn_sequence_gap("IMU", self.last_imu_sequence, message.sequence)
            self.last_imu_sequence = message.sequence
            self._broadcast_imu(message)
        elif isinstance(message, Statistics):
            LOGGER.info("telemetry statistics: %s", message)
        elif isinstance(message, TelemetryError):
            LOGGER.error("firmware rejected telemetry command with error 0x%02x", message.code)

    def _read_physical(self) -> None:
        assert self.serial_port is not None
        data = self.serial_port.read(self.serial_port.in_waiting or 1)
        if not data:
            return
        for frame in self.physical_decoder.feed(data):
            if frame.command == CMD_TELEMETRY:
                self._handle_telemetry(frame.payload)
                continue
            self._queue_rnode(frame.raw)
            if frame.command == CMD_RESET:
                self.capabilities = None
                self.configuration = None
                self.last_gps_sequence = None
                self.last_imu_sequence = None
                self.next_caps_query = time.monotonic() + 0.5

    def _read_rnode(self) -> None:
        try:
            data = os.read(self.rnode_master, 65536)
        except OSError as exc:
            if exc.errno == errno.EIO:
                return
            raise
        if not data:
            return
        # Hold partial host frames until their delimiter arrives. This is what
        # guarantees broker control traffic is inserted only between frames.
        for frame in self.host_decoder.feed(data):
            self._send_physical(frame.raw)

    def _accept_imu(self) -> None:
        assert self.imu_server is not None
        client, _address = self.imu_server.accept()
        client.setblocking(False)
        self.imu_clients.add(client)

    def run(self, stop_event: threading.Event | None = None) -> None:
        try:
            self._open()
            while stop_event is None or not stop_event.is_set():
                for key, _events in self.selector.select(timeout=0.05):
                    if key.data == "physical":
                        self._read_physical()
                    elif key.data == "rnode":
                        self._read_rnode()
                    elif key.data == "imu_server":
                        self._accept_imu()

                self._flush_fd(self.rnode_master, self.rnode_output)
                self._flush_fd(self.gps_master, self.gps_output)
                if time.monotonic() >= self.next_caps_query:
                    self._query_capabilities()
        finally:
            self.close()

    def close(self) -> None:
        for client in tuple(self.imu_clients):
            client.close()
        self.imu_clients.clear()
        for resource in (self.imu_server, self.serial_port):
            if resource is not None:
                try:
                    self.selector.unregister(resource)
                except (KeyError, ValueError):
                    pass
                resource.close()
        for fd in (self.rnode_master, self.rnode_slave, self.gps_master, self.gps_slave):
            if fd >= 0:
                try:
                    self.selector.unregister(fd)
                except (KeyError, ValueError):
                    pass
                os.close(fd)
        self.selector.close()
        for path, target in (
            (self.rnode_link, self.rnode_target),
            (self.gps_link, self.gps_target),
        ):
            if path.is_symlink() and target is not None and os.readlink(path) == target:
                path.unlink()
        if (
            self.imu_server is not None
            and self.imu_socket_path.exists()
            and stat.S_ISSOCK(self.imu_socket_path.lstat().st_mode)
        ):
            self.imu_socket_path.unlink()


def add_rnode_broker_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--port", required=True, help="physical RNode serial device")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--rnode-link", type=Path, default=Path("/run/rnode-gps/rnode"))
    parser.add_argument("--gps-link", type=Path, default=Path("/run/rnode-gps/gps"))
    parser.add_argument("--imu-socket", type=Path, default=Path("/run/rnode-gps/imu.sock"))
    parser.add_argument("--imu-rate", type=int, choices=SUPPORTED_IMU_RATES, default=50)
    parser.add_argument("--no-gps", action="store_true")
    parser.add_argument("--no-imu", action="store_true")
    parser.add_argument("--settle-time", type=float, default=1.0)
    parser.add_argument("--verbose", action="store_true")


def run_rnode_broker(args: argparse.Namespace) -> int:
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    broker = RNodeBroker(
        port=args.port,
        baudrate=args.baud,
        rnode_link=args.rnode_link,
        gps_link=args.gps_link,
        imu_socket=args.imu_socket,
        enable_gps=not args.no_gps,
        enable_imu=not args.no_imu,
        imu_rate_hz=args.imu_rate,
        settle_time_s=args.settle_time,
    )
    try:
        broker.run()
    except KeyboardInterrupt:
        return 0
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Multiplex one telemetry RNode serial link into RNode, GPS and IMU endpoints"
    )
    add_rnode_broker_arguments(parser)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return run_rnode_broker(args)
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
