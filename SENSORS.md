# Acquisition for visual-inertial and GNSS fusion

Build with the pinned ESP32 core 2.0.17 and SensorLib 0.3.3 using
`Tools/telemetry_firmware.sh build uart`. The UART application runs at 230400
baud. Install this revision's host broker with it. Neither the firmware nor
broker subtracts gyro bias or gravity. The display is optional in this build.

## IMU

The QMI8658 recipe retains ±4 g acceleration, ±512 deg/s angular rate,
`ACC_ODR_125Hz` / `GYR_ODR_112_1Hz`, and LPF modes 0 / 3. These enum names are
nominal settings, not measured rates or delays. The 100 Hz publication target
now sends every native locked sample, nominally about 112 Hz. Lower requested
rates are deliberate decimation for diagnostics and are marked unsuitable for
lossless fusion. Verify actual cadence and configure the consumer accordingly.

The worker polls readiness at one FreeRTOS tick, uses the QMI locking protocol,
waits at least 12 µs, checks Locked, then reads the complete counter, temperature,
accelerometer and gyro register block. Reading GZ_H releases the lock. This
avoids mixed-axis epochs and the pinned SensorLib timestamp unwrap bug.
No sensor FIFO is enabled, and there are no fabricated FIFO timestamps.
The hardware 24-bit sample count detects missed native samples. Queue overflow
and lock failures remain visible, even if an affected frame cannot be delivered.

New subtype `0x21` appends to the legacy IMU body (big endian):

| Field | Type | Meaning |
|---|---|---|
| `sensor_counter` | u32 | Raw wrapping 24-bit QMI sample count; not time |
| `ready_lower_us` | u64 | Start of last SPI poll observing no available data; zero if unknown |
| `ready_upper_us` | u64 | End of first SPI poll observing available data |
| `read_completed_us` | u64 | End of locked register read |

The original `device_time_us` is the bracket midpoint, or the upper observation
when no bracket is available (invalid timing flag). These ESP monotonic bounds
describe **data readiness**, not a calibrated physical sensing instant. The
host must account for filter/group delay and validate its ESP-to-Pi mapping.
Never use UART/socket receipt as capture time or assume a 5 ms total error.
Sensor axes are native register X/Y/Z; measure the installed board-to-robot and
camera-to-IMU transforms, without guessing signs from board placement.

| Status bit | Meaning |
|---|---|
| 0 / 1 / 2 | Accelerometer / gyro / temperature data present |
| 3 | At least one inertial axis within 0.1% of full scale |
| 4 | Missed sensor sample or failed acquisition |
| 5 | Firmware queue overflow since the last successfully queued sample |
| 6 | Unbounded/failed acquisition timing, counter duplicate/reset, or stream restart |
| 7 | Intentionally decimated diagnostic stream |

Only status 3 or 7 can qualify for fusion, with independent clock/calibration,
sequence, hardware counter and timing checks. Invalid rows are still recorded.
The first sample after enabling is invalid. A missing bracket cannot silently
become a zero-width interval. Saturation uses raw ADC rails before unit rounding.
Temperature is centi-C; acceleration mg and angular rate mdps retain their
existing scale and sign. Integer conversions handle negative rails safely.

## GNSS

Firmware already forwards every checksum-valid receiver sentence unchanged.
The host pairs GGA altitude/geoid separation, RMC/ZDA date/time, and any accuracy
message by the receiver epoch. Its existing display median is not a fusion input.
For L76K, the published protocol does **not** document GST or a per-solution
position covariance message. HDOP is insufficient for a covariance in metres.
Unknown covariance must remain unknown; this receiver cannot yet be declared
ready for covariance-gated GNSS fusion. A u-blox option needs its own verified
receiver protocol/configuration. Identify the fitted device before configuring it.

Keep the supported 1 Hz receiver output for now. The L76K spec conditions faster
output on serial bandwidth/output restrictions, so this change does not blindly
request 5 Hz or discard sentences needed to reconstruct an epoch. PPS correlation
and a measured receiver-epoch-to-Pi mapping remain acceptance work. The ESP NMEA
timestamp measures forwarding, not the receiver epoch; no timing claim is inferred
from it. Receiver filter settings, antenna transform and accuracy evidence belong
in the robot's calibration profile.

## Validation and deployment

Run `Tools/test_telemetry_protocol.sh` and
`python3 -m unittest discover -s Host -t . -p 'test_*.py'`, then build the UART
image. Tests cover counter wrap/gaps/resets, queue-fault retention, saturation,
signed conversion, malformed acquisition metadata and broker transport recovery.
They do not prove board timing, sustained throughput, filter delay or axis mounting.

Use the [Pi UART flashing procedure](README.md#uart-firmware-flash-through-the-pi-keeping-gpio-wiring).
Crawler0's configured Pi 5 UART is `/dev/ttyAMA0`; BOOT/RESET is manual with the
current wiring. Preserve EEPROM and verify the copied image checksum. Record
the flashed revision, file SHA-256, embedded application hash, receiver identity,
actual IMU cadence, drops and timing evidence. Take a new undisturbed stationary
window after warm-up; OpenVINS/MINS estimate bias from raw samples at each start.
Do not reuse the previous shaken-table capture as a stationary calibration.

Primary references:

- [QST QMI8658C register and locking protocol](https://files.waveshare.com/wiki/common/QMI8658C_datasheet_rev_0.9.pdf)
- [SensorLib source](https://github.com/lewisxhe/SensorLib)
- [Quectel L76K protocol specification, mirrored by Seeed](https://raw.githubusercontent.com/Seeed-Projects/Seeed_L76K-GNSS_for_XIAO/fb74b715224e0ac153c3884e578ee8e024ed8946/docs/Quectel_L76K_GNSS_Protocol_Specification_V1.1.pdf)
- [Espressif boot mode selection](https://docs.espressif.com/projects/esptool/en/latest/esp32s3/advanced-topics/boot-mode-selection.html)
