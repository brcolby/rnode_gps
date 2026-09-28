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

This installs ESP32 Arduino core 2.0.17 and exact versions of every direct and
transitive library dependency. The versions are locked in `Makefile`, including
SensorLib 0.3.3. The build wrapper stages the sketch and intermediate files in
a correctly named temporary directory, so it works regardless of the checkout
directory name and does not reuse an implicit Arduino build cache. Compiler
prefix maps remove the random staging path and checkout path from ELF debug
data, making clean build artifacts byte-identical when the locked inputs are
unchanged.

## Build

First reproduce the unmodified T-Beam Supreme baseline:

```bash
./Tools/telemetry_firmware.sh build stock
# Equivalent: make firmware-tbeam_supreme
```

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

Artifacts are written to `build/stock-tbeam-supreme`, `build/telemetry-usb`,
or `build/telemetry-uart`.

After building all three variants, capture the complete toolchain, library,
artifact-size, and SHA-256 manifest with:

```bash
./Tools/firmware_artifact_manifest.sh
```

## Flash

Attach the LoRa antenna before powering the board. Stop Reticulum and the host
broker first, because the flashing tool needs exclusive access to the device.
The wrapper also uses `rnodeconf` after upload to provision the ESP application
hash retained by RNode's firmware-integrity check. Set `RNODECONF` to an
explicit executable path if `rnodeconf` is not on `PATH`.
Find the USB programming port:

```bash
arduino-cli board list
```

Then build and flash in one command, substituting the displayed port:

```bash
# Linux example
./Tools/telemetry_firmware.sh flash usb /dev/ttyACM0

# macOS example
RNODECONF=/Users/bcolby/projects/gestalt/.venv/bin/rnodeconf \
  ./Tools/telemetry_firmware.sh flash usb /dev/cu.usbmodem101
```

The wrapper prints `Firmware hash provisioned` after the runtime device has
accepted the new image hash. Do not proceed with hardware tests if upload
succeeds but this final provisioning step fails. Reflashing a telemetry build
over another telemetry build is supported and does not require restoring stock
first; normal ESP32 application upload preserves the EEPROM partition.

Use `flash uart` instead of `flash usb` when the runtime host connection will
be UART0. Connect the 3.3 V USB-UART adapter before flashing, then provide both
the board's USB programming port and the adapter's UART runtime port:

```bash
./Tools/telemetry_firmware.sh flash uart \
  /dev/cu.usbmodem101 /dev/cu.usbserial-0001
```

The board is still programmed through its USB connector, while `rnodeconf`
provisions the firmware hash over the UART adapter after boot. Connect host RX
to GPIO 43, host TX to GPIO 44, and host ground to board ground. Do not apply
5 V logic to these pins.

If automatic bootloader entry fails, hold BOOT, tap RESET, start the flash,
then release BOOT when the upload begins. A first hardware trial should use
the USB build because it needs no external serial wiring.

## Host setup

The broker and Reticulum configuration are documented in
[Host/README.md](Host/README.md). On Linux, use the default `/run` endpoints:

```bash
python3 -m venv .venv
.venv/bin/pip install -r Host/requirements.txt
sudo .venv/bin/python Host/rnode_broker.py --port /dev/ttyACM0
```

Reticulum opens `/run/rnode-gps/rnode`; GPS consumers open
`/run/rnode-gps/gps`; IMU consumers connect to
`/run/rnode-gps/imu.sock`.

On macOS, avoid root-owned `/run` paths and start the broker explicitly with
writable endpoints:

```bash
/Users/bcolby/projects/gestalt/.venv/bin/python Host/rnode_broker.py \
  --port /dev/cu.usbmodem101 \
  --rnode-link /tmp/rnode-gps/rnode \
  --gps-link /tmp/rnode-gps/gps \
  --imu-socket /tmp/rnode-gps/imu.sock \
  --imu-rate 100
```

The broker must remain running while consumers use those endpoints. It is the
only process that may open the physical serial port; stop it before flashing,
running `rnodeconf`, or performing direct serial validation.

## KISS extension

Telemetry uses private command `0xA0`. A complete frame is `C0 A0 PAYLOAD C0`,
where `PAYLOAD` uses normal KISS escaping (`DB DC` for `C0`, `DB DD` for
`DB`). Unknown escape pairs, a dangling escape, an oversized host command, or
an integrity failure make the telemetry command invalid; they never change
radio state. Ordinary non-telemetry KISS frames retain their original bytes.

The unescaped payload is:

```
version u8 | subtype u8 | body | crc16 u16
```

Version is `0x01`. All integers, including the trailing CRC, are big-endian.
The CRC is CRC-16/CCITT-FALSE over version, subtype, and body: polynomial
`0x1021`, initial value `0xffff`, no reflection, and no final XOR. Its standard
`123456789` check value is `0x29b1`. KISS delimiters, the `0xA0` command byte,
and escape bytes are not part of the CRC.

| Subtype | Direction | Body after version/subtype | Body bytes | Total payload bytes |
| --- | --- | --- | ---: | ---: |
| `0x00` capabilities query | host to device | empty | 0 | 4 |
| `0x01` capabilities | device to host | supported flags, ready flags, rate mask, RNode major, RNode minor | 5 | 9 |
| `0x02` configure | host to device | enable flags, IMU rate in Hz | 2 | 6 |
| `0x03` configuration | device to host | active flags, IMU rate, ready flags | 3 | 7 |
| `0x04` statistics query | host to device | empty | 0 | 4 |
| `0x05` statistics | device to host | GPS seq u16, IMU seq u16, invalid NMEA u32, GPS drops u32, IMU drops u32 | 16 | 20 |
| `0x10` GPS NMEA | device to host | seq u16, monotonic microseconds u64, NMEA bytes without CR/LF | 10 + N | 14 + N |
| `0x20` IMU sample | device to host | seq u16, monotonic microseconds u64, accel 3xi16 mg, gyro 3xi32 mdps, temperature i16 centi-C, status u8 | 31 | 35 |
| `0x30` GNSS NAVX query | host to device | empty | 0 | 4 |
| `0x31` GNSS NAVX state | device to host | status u8, followed by the raw 44-byte little-endian CASIC CFG-NAVX payload on success | 1 or 45 | 5 or 49 |
| `0x32` GNSS dynamic-model set | host to device | documented CFG-NAVX dynamic-model value u8 (0 through 7) | 1 | 5 |
| `0x33` GNSS dynamic-model state | device to host | status u8 | 1 | 5 |
| `0x7f` error | device to host | error code | 1 | 5 |

Enable flag bit 0 is GPS and bit 1 is IMU. Supported IMU rates are 10, 25,
50, and 100 Hz. Axis values use the QMI8658's native board orientation; frame
conversion and sensor fusion belong on the host.

The capabilities rate mask uses bits 0 through 3 for 10, 25, 50, and 100 Hz.
Unknown flag and rate-mask bits must be ignored. The configuration response is
authoritative: it reports the subset that is both requested and ready. GPS
NMEA is 7 to 128 ASCII bytes, starts with `$` or `!`, ends immediately after
two checksum hex digits, and excludes CR/LF. The maximum unescaped telemetry
payload is therefore 142 bytes.

Error codes are `0x01` unsupported version, `0x02` invalid length or value,
`0x03` unsupported subtype, and `0x04` malformed framing or bad CRC. A host
must treat unknown error codes as failures. The firmware can answer a damaged
request with an integrity error, but the host never assumes that an error
response will survive the same faulty link.

The NAVX query is read-only and never saves or changes receiver configuration.
The dynamic-model setter applies only CFG-NAVX mask bit 0 and does not issue
CFG-CFG, so it does not save the change to receiver nonvolatile storage.
Its status is `0x00` success, `0x01` timeout, `0x02` receiver NACK, `0x03`
invalid CASIC response, or `0x04` query already in progress. The L76K-specific
public protocol does not promise CFG-NAVX support, so timeout or NACK is a valid
hardware result rather than a telemetry transport failure.

## Session and recovery contract

Telemetry is disabled on every boot. After the stock RNode startup/reset
frame, or after opening a link with unknown state, the broker repeatedly sends
a capabilities query. It sends configuration only after a valid capabilities
response and considers negotiation complete only after a valid configuration
response matches the requested rate and the ready subset. A stock RNode does
not understand `0xA0`; timeout is therefore a clean capability failure, not
permission to pass telemetry bytes into Reticulum.

A device reset clears enabled flags, sequence numbers, counters, and the
device monotonic epoch. The broker clears its corresponding session state and
renegotiates. Duplicate capabilities or configuration frames are idempotent.
Malformed or corrupt telemetry frames are discarded. Radio frames before,
during, and after renegotiation continue through the broker.

GPS and IMU sequences start at zero, increment for every attempted emission,
and wrap from `65535` to zero. Thus a modulo-65536 gap exposes device-side
backpressure drops as well as link loss. The five statistics values are
snapshots; 32-bit counters also wrap modulo their field width. Sequence
rollover is not a reset. Only a reset indication or a backwards device clock
that cannot belong to the current epoch starts a new session.

`device_time_us` is the ESP32 monotonic microsecond clock sampled when a valid
NMEA sentence is emitted or immediately before an IMU read. It is not GNSS
UTC, PPS-disciplined time, or the exact sensor conversion instant. GNSS UTC
remains inside applicable NMEA sentences. Host records may add receive wall
time, but consumers must retain the device timestamp and session boundary
rather than treating host arrival time as sensor time.

Telemetry is best-effort and cannot reserve space needed by radio output. The
firmware emits a telemetry frame only when the complete worst-case escaped
frame fits the serial transmit buffer; otherwise it increments the relevant
drop counter. This makes frame insertion atomic at the firmware loop level and
prevents a sensor sample from blocking ordinary RNode work.

## Conformance vectors

[`Host/protocol_vectors.json`](Host/protocol_vectors.json) is the canonical
machine-readable set of payload and full KISS frames, including boundary
integers, sequence rollover values, escaping, and invalid inputs. Run both
implementations against it before building firmware:

```bash
./Tools/test_telemetry_protocol.sh
PYTHONPYCACHEPREFIX=/tmp/rnode-gps-pycache \
  python3 -m unittest -v Host.test_rnode_broker.ProtocolVectorTest
```

The first command compiles the Arduino-independent C++ primitives used by the
firmware, including the actual command state machine and bounded telemetry
frame parser; the second verifies the Python host codec and KISS encoding.
`make test-telemetry` runs both suites when Python has pyserial installed.

For sustained deterministic PTY multiplexing with corruption, fragmentation,
resets, rollover, delayed consumers, queue measurements, and cleanup evidence:

```bash
make telemetry-soak
make telemetry-soak SOAK_CYCLES=4096
```

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
