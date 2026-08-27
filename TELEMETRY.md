# T-Beam Supreme: RNode, GPS and IMU over one serial link

This fork adds opt-in, wired-only firmware for the LilyGO T-Beam Supreme with
an SX1262 radio. Normal RNode KISS traffic, checksum-valid GPS NMEA sentences,
and raw QMI8658 samples share one serial connection. A small host broker
removes the extension frames before passing ordinary RNode traffic to an
unchanged Reticulum `RNodeInterface`.

The stock RNode targets and behavior remain unchanged. The custom targets
disable Wi-Fi, BLE, and the on-device console so only one host transport owns
the KISS stream. GPS and IMU streaming stay off until the broker negotiates
the extension.

## Prerequisites

Install Arduino CLI. On macOS with Homebrew:

```bash
brew install arduino-cli
```

From the repository root, install the pinned ESP32 core and libraries:

```bash
make prep-esp32
```

This installs ESP32 Arduino core 2.0.17 and SensorLib 0.3.3. The build wrapper
stages the sketch in a correctly named temporary directory, so it works even
though this repository is checked out as `rnode_gps`.

## Build

For the native ESP32-S3 USB CDC connection:

```bash
./Tools/telemetry_firmware.sh build usb
# Equivalent: make firmware-tbeam_supreme-telemetry-usb
```

For a 3.3 V TTL host on UART0, GPIO 43 TX and GPIO 44 RX:

```bash
./Tools/telemetry_firmware.sh build uart
# Equivalent: make firmware-tbeam_supreme-telemetry-uart
```

Artifacts are written to `build/telemetry-usb` or `build/telemetry-uart`.

## Flash

Attach the LoRa antenna before powering the board. Stop Reticulum and the host
broker first, because the flashing tool needs exclusive access to the device.
Find the USB programming port:

```bash
arduino-cli board list
```

Then build and flash in one command, substituting the displayed port:

```bash
# Linux example
./Tools/telemetry_firmware.sh flash usb /dev/ttyACM0

# macOS example
./Tools/telemetry_firmware.sh flash usb /dev/cu.usbmodem101
```

Use `flash uart` instead of `flash usb` when the runtime host connection will
be UART0. The board is still programmed through its USB connector. After a
UART build boots, connect host RX to GPIO 43, host TX to GPIO 44, and host
ground to board ground. Do not apply 5 V logic to these pins.

If automatic bootloader entry fails, hold BOOT, tap RESET, start the flash,
then release BOOT when the upload begins. A first hardware trial should use
the USB build because it needs no external serial wiring.

## Host setup

The broker and Reticulum configuration are documented in
[Host/README.md](Host/README.md). In brief:

```bash
python3 -m venv .venv
.venv/bin/pip install -r Host/requirements.txt
sudo .venv/bin/python Host/rnode_broker.py --port /dev/ttyACM0
```

Reticulum opens `/run/rnode-gps/rnode`; GPS consumers open
`/run/rnode-gps/gps`; IMU consumers connect to
`/run/rnode-gps/imu.sock`.

## KISS extension

Telemetry uses command `0xA0`. Its payload is KISS-escaped like every other
RNode command and begins with protocol version `0x01` followed by a subtype.
All integer fields are big-endian.

| Subtype | Direction | Payload after version/subtype |
| --- | --- | --- |
| `0x00` capabilities query | host to device | empty |
| `0x01` capabilities | device to host | supported flags, ready flags, rate mask, RNode major, RNode minor |
| `0x02` configure | host to device | enable flags, IMU rate in Hz |
| `0x03` configuration | device to host | active flags, IMU rate, ready flags |
| `0x04` statistics query | host to device | empty |
| `0x05` statistics | device to host | GPS seq u16, IMU seq u16, invalid NMEA u32, GPS drops u32, IMU drops u32 |
| `0x10` GPS NMEA | device to host | seq u16, monotonic microseconds u64, NMEA bytes without CR/LF |
| `0x20` IMU sample | device to host | seq u16, monotonic microseconds u64, accel 3xi16 mg, gyro 3xi32 mdps, temperature i16 centi-C, status u8 |
| `0x7f` error | device to host | error code |

Enable flag bit 0 is GPS and bit 1 is IMU. Supported IMU rates are 10, 25,
50, and 100 Hz. Axis values use the QMI8658's native board orientation; frame
conversion and sensor fusion belong on the host.

## Hardware validation checklist

Compilation and broker tests cannot prove the board wiring or sensor-library
behavior. On the first flash, verify:

1. The broker reports both capability bits as supported and ready.
2. Reticulum connects through `/run/rnode-gps/rnode` and can transmit and
   receive with the broker running.
3. `/run/rnode-gps/gps` emits valid NMEA and produces a fix outdoors.
4. `/run/rnode-gps/imu.sock` emits changing JSON samples as the board moves.
5. Sustained LoRa traffic at the intended IMU rate produces no sequence-gap
   or queue warnings.

Keep a known-good upstream firmware image available for recovery.
