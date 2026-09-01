# RNode/GPS/IMU host broker

`rnode_broker.py` is the only process that opens the physical T-Beam serial
device. It preserves ordinary RNode KISS frames byte-for-byte and consumes
only this fork's command `0xA0` frames. It exposes:

- `/run/rnode-gps/rnode`: PTY for an unchanged Reticulum `RNodeInterface`.
- `/run/rnode-gps/gps`: PTY containing checksum-valid NMEA with CR/LF endings.
- `/run/rnode-gps/imu.sock`: multi-client Unix stream socket containing JSONL.

## Run manually

From the repository root:

```bash
python3 -m venv .venv
.venv/bin/pip install -r Host/requirements.txt
sudo .venv/bin/python Host/rnode_broker.py --port /dev/ttyACM0
```

Root is only needed when creating endpoints under `/run`. For development,
use writable paths and no `sudo`:

```bash
.venv/bin/python Host/rnode_broker.py \
  --port /dev/cu.usbmodem101 \
  --rnode-link /tmp/rnode-gps/rnode \
  --gps-link /tmp/rnode-gps/gps \
  --imu-socket /tmp/rnode-gps/imu.sock
```

The broker requests GPS plus 50 Hz IMU by default. Use `--no-gps`, `--no-imu`,
or `--imu-rate {10,25,50,100}` to change that. `--verbose` enables debug logs.

## Reticulum

Point a standard RNode interface at the broker's RNode PTY:

```ini
[[T-Beam-Supreme]]
  type = RNodeInterface
  enabled = yes
  port = /run/rnode-gps/rnode
  frequency = 915000000
  bandwidth = 125000
  txpower = 10
  spreadingfactor = 8
  codingrate = 5
```

Set the radio parameters to values legal for your location and matching the
rest of your network. Start the broker before `rnsd`.

## GPS and IMU consumers

Any NMEA reader can open the GPS PTY at its normal serial settings. The baud
setting on a PTY is ignored, but 9600 is a conventional choice:

```bash
cat /run/rnode-gps/gps
```

Read newline-delimited IMU JSON with `socat`:

```bash
socat - UNIX-CONNECT:/run/rnode-gps/imu.sock
```

IMU records contain the raw integer units (`accel_mg`, `gyro_mdps`), converted
SI values, temperature, device monotonic time, host wall-clock time, sequence,
and sensor status.

Sensor consumers are best-effort and cannot block RNode traffic. The primary
RNode PTY instead has a bounded 1 MiB queue and fails loudly if Reticulum stops
reading it.

## systemd example

The included `rnode-gps-broker.service` assumes the checkout and virtual
environment are at `/opt/rnode_gps`, and the device is `/dev/ttyACM0`. Adjust
those paths first if needed, then:

```bash
sudo install -m 0644 Host/rnode-gps-broker.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now rnode-gps-broker.service
```

Review logs with `journalctl -u rnode-gps-broker.service`. Stop the broker
before flashing or running `rnodeconf`, since those tools must open the
physical device directly.

## Test without hardware

```bash
PYTHONPYCACHEPREFIX=/tmp/rnode-gps-pycache \
  .venv/bin/python -m unittest -v Host.test_rnode_broker
./Tools/test_telemetry_protocol.sh
```

The integration test creates a pseudo-terminal as a simulated physical RNode
and verifies simultaneous bidirectional RNode traffic, NMEA output, and IMU
JSON output over one link. The standalone C++ test and the Python vector tests
verify the CRC-protected private wire contract against
`Host/protocol_vectors.json`.
