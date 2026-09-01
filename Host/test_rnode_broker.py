from __future__ import annotations

import json
import os
import pty
import select
import socket
import struct
import tempfile
import threading
import time
import tty
import unittest
from pathlib import Path

from Host.rnode_broker import (
    CMD_TELEMETRY,
    FEND,
    FESC,
    GPS_NMEA,
    IMU_SAMPLE,
    PROTOCOL_VERSION,
    STATS_RESPONSE,
    TFEND,
    TFESC,
    GPSMessage,
    IMUMessage,
    KissStreamDecoder,
    RNodeBroker,
    Statistics,
    crc16_ccitt,
    decode_telemetry,
    encode_kiss,
    encode_telemetry,
)


PROTOCOL_VECTORS = Path(__file__).with_name("protocol_vectors.json")


class KissCodecTest(unittest.TestCase):
    def test_round_trips_fragmented_escaped_frame(self) -> None:
        payload = bytes((0x01, FEND, 0x02, FESC, 0x03))
        encoded = encode_kiss(CMD_TELEMETRY, payload)
        self.assertEqual(encoded, bytes((FEND, CMD_TELEMETRY, 1, FESC, TFEND, 2, FESC, TFESC, 3, FEND)))

        decoder = KissStreamDecoder()
        frames = []
        for byte in encoded:
            frames.extend(decoder.feed(bytes((byte,))))
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].command, CMD_TELEMETRY)
        self.assertEqual(frames[0].payload, payload)
        self.assertEqual(frames[0].raw, encoded)

    def test_preserves_back_to_back_standard_frames(self) -> None:
        first = encode_kiss(0x01, b"abcd")
        second = encode_kiss(0x00, b"radio")
        frames = list(KissStreamDecoder().feed(first + second))
        self.assertEqual([frame.raw for frame in frames], [first, second])


class TelemetryCodecTest(unittest.TestCase):
    def test_crc_matches_ccitt_false_check_value(self) -> None:
        self.assertEqual(crc16_ccitt(b"123456789"), 0x29B1)

    def test_decodes_gps(self) -> None:
        payload = encode_telemetry(
            GPS_NMEA, struct.pack(">HQ", 7, 123456) + b"$GPRMC,example*00"
        )
        message = decode_telemetry(payload)
        self.assertIsInstance(message, GPSMessage)
        assert isinstance(message, GPSMessage)
        self.assertEqual(message.sequence, 7)
        self.assertEqual(message.device_time_us, 123456)
        self.assertEqual(message.sentence, "$GPRMC,example*00")

    def test_decodes_imu_and_emits_jsonl(self) -> None:
        body = struct.pack(">HQhhhiiihB", 9, 987654, 100, -200, 1000, 1001, -2002, 3003, 2450, 7)
        message = decode_telemetry(encode_telemetry(IMU_SAMPLE, body))
        self.assertIsInstance(message, IMUMessage)
        assert isinstance(message, IMUMessage)
        self.assertEqual(message.accel_mg, (100, -200, 1000))
        self.assertEqual(message.gyro_mdps, (1001, -2002, 3003))
        record = json.loads(message.as_json())
        self.assertAlmostEqual(record["temperature_c"], 24.5)
        self.assertAlmostEqual(record["accel_m_s2"]["z"], 9.80665)

    def test_rejects_wrong_length(self) -> None:
        with self.assertRaises(ValueError):
            decode_telemetry(encode_telemetry(IMU_SAMPLE, b"\x00"))

    def test_rejects_corrupt_crc(self) -> None:
        payload = bytearray(encode_telemetry(GPS_NMEA, struct.pack(">HQ", 1, 2) + b"$G*00"))
        payload[4] ^= 0x01
        with self.assertRaisesRegex(ValueError, "CRC mismatch"):
            decode_telemetry(bytes(payload))

    def test_decodes_statistics(self) -> None:
        body = struct.pack(">HHIII", 12, 34, 56, 78, 90)
        message = decode_telemetry(encode_telemetry(STATS_RESPONSE, body))
        self.assertEqual(message, Statistics(12, 34, 56, 78, 90))


class ProtocolVectorTest(unittest.TestCase):
    def test_golden_vectors_match_payload_crc_and_kiss_encoding(self) -> None:
        document = json.loads(PROTOCOL_VECTORS.read_text(encoding="utf-8"))
        for vector in document["vectors"]:
            with self.subTest(vector=vector["name"]):
                subtype = int(vector["subtype"], 16)
                body = bytes.fromhex(vector["body_hex"])
                payload = bytes.fromhex(vector["payload_hex"])
                self.assertEqual(encode_telemetry(subtype, body), payload)
                self.assertEqual(encode_kiss(CMD_TELEMETRY, payload).hex(), vector["kiss_hex"])
                self.assertEqual(crc16_ccitt(payload[:-2]), int.from_bytes(payload[-2:], "big"))

    def test_device_response_vectors_decode(self) -> None:
        document = json.loads(PROTOCOL_VECTORS.read_text(encoding="utf-8"))
        device_subtypes = {0x01, 0x03, 0x05, 0x10, 0x20, 0x7F}
        for vector in document["vectors"]:
            subtype = int(vector["subtype"], 16)
            if subtype in device_subtypes:
                with self.subTest(vector=vector["name"]):
                    decode_telemetry(bytes.fromhex(vector["payload_hex"]))

    def test_invalid_vectors_fail_with_declared_host_error(self) -> None:
        document = json.loads(PROTOCOL_VECTORS.read_text(encoding="utf-8"))
        for vector in document["invalid_vectors"]:
            with self.subTest(vector=vector["name"]):
                with self.assertRaisesRegex(ValueError, vector["host_error"]):
                    decode_telemetry(bytes.fromhex(vector["payload_hex"]))

    def test_reset_session_vectors_are_complete_kiss_frames(self) -> None:
        document = json.loads(PROTOCOL_VECTORS.read_text(encoding="utf-8"))
        reset, first_query = document["session_vectors"]
        reset_frames = list(KissStreamDecoder().feed(bytes.fromhex(reset["kiss_hex"])))
        self.assertEqual([(frame.command, frame.payload) for frame in reset_frames], [(0x55, b"\xf8")])
        self.assertEqual(first_query["kiss_hex"], encode_kiss(CMD_TELEMETRY, encode_telemetry(0x00)).hex())


class BrokerIntegrationTest(unittest.TestCase):
    @staticmethod
    def _read_frame(fd: int, decoder: KissStreamDecoder, command: int, timeout: float = 3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            readable, _, _ = select.select([fd], [], [], deadline - time.monotonic())
            if not readable:
                break
            for frame in decoder.feed(os.read(fd, 65536)):
                if frame.command == command:
                    return frame
        raise AssertionError(f"timed out waiting for KISS command 0x{command:02x}")

    @staticmethod
    def _wait_for(predicate, timeout: float = 3.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.01)
        raise AssertionError("timed out waiting for broker state")

    def test_routes_all_three_streams_over_one_physical_pty(self) -> None:
        physical_master, physical_slave = pty.openpty()
        tty.setraw(physical_slave)
        physical_path = os.ttyname(physical_slave)
        os.close(physical_slave)
        os.set_blocking(physical_master, False)

        stop = threading.Event()
        errors: list[BaseException] = []
        rnode_fd = -1
        gps_fd = -1
        imu_client: socket.socket | None = None

        with tempfile.TemporaryDirectory(dir="/private/tmp") as directory:
            runtime = Path(directory)
            broker = RNodeBroker(
                port=physical_path,
                rnode_link=runtime / "rnode",
                gps_link=runtime / "gps",
                imu_socket=runtime / "imu.sock",
                settle_time_s=0.01,
            )

            def run_broker() -> None:
                try:
                    broker.run(stop)
                except BaseException as exc:  # surfaced on the test thread below
                    errors.append(exc)

            thread = threading.Thread(target=run_broker, daemon=True)
            thread.start()

            try:
                self._wait_for(lambda: errors or (runtime / "rnode").exists())
                self.assertEqual(errors, [])
                device_decoder = KissStreamDecoder()
                query = self._read_frame(physical_master, device_decoder, CMD_TELEMETRY)
                self.assertEqual(query.payload, encode_telemetry(0x00))

                capabilities = encode_telemetry(0x01, bytes((0x03, 0x03, 0x0F, 0x01, 0x56)))
                os.write(physical_master, encode_kiss(CMD_TELEMETRY, capabilities))
                configure = self._read_frame(physical_master, device_decoder, CMD_TELEMETRY)
                self.assertEqual(configure.payload, encode_telemetry(0x02, bytes((0x03, 50))))

                configured = encode_telemetry(0x03, bytes((0x03, 50, 0x03)))
                os.write(physical_master, encode_kiss(CMD_TELEMETRY, configured))
                self._wait_for(lambda: broker.configuration is not None)

                rnode_fd = os.open(runtime / "rnode", os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                gps_fd = os.open(runtime / "gps", os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                imu_client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                imu_client.settimeout(3.0)
                imu_client.connect(str(runtime / "imu.sock"))
                self._wait_for(lambda: len(broker.imu_clients) == 1)

                radio_frame = encode_kiss(0x23, b"\x80")
                gps_payload = (
                    encode_telemetry(
                        GPS_NMEA,
                        struct.pack(">HQ", 1, 1000) + b"$GPGGA,example*00",
                    )
                )
                os.write(physical_master, radio_frame + encode_kiss(CMD_TELEMETRY, gps_payload))
                forwarded = self._read_frame(rnode_fd, KissStreamDecoder(), 0x23)
                self.assertEqual(forwarded.raw, radio_frame)

                readable, _, _ = select.select([gps_fd], [], [], 3.0)
                self.assertTrue(readable)
                self.assertEqual(os.read(gps_fd, 256), b"$GPGGA,example*00\r\n")

                imu_body = struct.pack(">HQhhhiiihB", 2, 2000, 1, 2, 1000, 4, 5, 6, 2500, 7)
                os.write(
                    physical_master,
                    encode_kiss(CMD_TELEMETRY, encode_telemetry(IMU_SAMPLE, imu_body)),
                )
                imu_record = json.loads(imu_client.recv(4096))
                self.assertEqual(imu_record["sequence"], 2)
                self.assertEqual(imu_record["accel_mg"]["z"], 1000)

                host_frame = encode_kiss(0x01, b"host-to-radio")
                os.write(rnode_fd, host_frame)
                outbound = self._read_frame(physical_master, device_decoder, 0x01)
                self.assertEqual(outbound.raw, host_frame)
            finally:
                stop.set()
                thread.join(3.0)
                if imu_client is not None:
                    imu_client.close()
                if rnode_fd >= 0:
                    os.close(rnode_fd)
                if gps_fd >= 0:
                    os.close(gps_fd)
                os.close(physical_master)

            self.assertFalse(thread.is_alive())
            self.assertEqual(errors, [])


if __name__ == "__main__":
    unittest.main()
