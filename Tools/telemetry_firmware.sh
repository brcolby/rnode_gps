#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  Tools/telemetry_firmware.sh build {stock|usb|uart}
  Tools/telemetry_firmware.sh flash {stock|usb} USB_PORT
  Tools/telemetry_firmware.sh flash uart USB_PORT UART_PORT

Examples:
  Tools/telemetry_firmware.sh build usb
  Tools/telemetry_firmware.sh flash usb /dev/ttyACM0
  Tools/telemetry_firmware.sh flash uart /dev/ttyACM0 /dev/ttyUSB0
EOF
}

if [[ $# -lt 2 || $# -gt 4 ]]; then
  usage >&2
  exit 2
fi

action=$1
transport=$2
port=${3:-}
runtime_port=${4:-$port}

if [[ $action != build && $action != flash ]]; then
  usage >&2
  exit 2
fi
if [[ $action == build && $# -ne 2 ]]; then
  echo "Build does not accept serial-port arguments." >&2
  usage >&2
  exit 2
fi
if [[ $transport != stock && $transport != usb && $transport != uart ]]; then
  usage >&2
  exit 2
fi
if [[ $action == flash && -z $port ]]; then
  echo "A serial port is required when flashing." >&2
  usage >&2
  exit 2
fi
if [[ $action == flash && $transport == uart && $# -ne 4 ]]; then
  echo "UART flashing requires the USB programming port and UART runtime port." >&2
  usage >&2
  exit 2
fi
if [[ $action == flash && $transport != uart && $# -eq 4 ]]; then
  echo "A separate runtime port is only valid for the UART transport." >&2
  usage >&2
  exit 2
fi

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
build_tmp_root=${TMPDIR:-/tmp}
build_tmp_root=${build_tmp_root%/}
stage_root=$(mktemp -d "$build_tmp_root/rnode-gps-build.XXXXXX")
stage_root_physical=$(cd "$stage_root" && pwd -P)
sketch_dir="$stage_root/RNode_Firmware"
output_dir="$repo_root/build/telemetry-$transport"
if [[ $transport == stock ]]; then
  output_dir="$repo_root/build/stock-tbeam-supreme"
fi

cleanup() {
  rm -rf -- "$stage_root"
}
trap cleanup EXIT

mkdir -p "$sketch_dir" "$output_dir"
sources=("$repo_root"/*.ino "$repo_root"/*.h "$repo_root"/*.cpp)
cp "${sources[@]}" "$sketch_dir/"

cli=("${ARDUINO_CLI:-arduino-cli}")
if [[ -n ${ARDUINO_CLI_CONFIG_DIR:-} ]]; then
  cli+=(--config-dir "$ARDUINO_CLI_CONFIG_DIR")
fi
cli+=(--config-file "${ARDUINO_CLI_CONFIG:-$repo_root/arduino-cli.yaml}")

fqbn="esp32:esp32:esp32s3:CDCOnBoot=cdc"
extra_flags='"-DBOARD_MODEL=0x3D"'
if [[ $transport != stock ]]; then
  extra_flags+=' "-DRNODE_GPS_TELEMETRY=1" "-DRNODE_GPS_WIRED_ONLY=1"'
fi
if [[ $transport == uart ]]; then
  fqbn="esp32:esp32:esp32s3:CDCOnBoot=default"
  extra_flags+=' "-DRNODE_GPS_HOST_UART=1"'
fi

# Arduino embeds source and build paths in ELF debug data, then stores the ELF
# digest in the ESP application image. Map the random staging directory to one
# stable logical path so clean builds produce byte-identical artifacts.
path_map_flags="\"-ffile-prefix-map=$stage_root=/rnode-build\" \"-fdebug-prefix-map=$stage_root=/rnode-build\""
if [[ $stage_root_physical != "$stage_root" ]]; then
  path_map_flags+=" \"-ffile-prefix-map=$stage_root_physical=/rnode-build\" \"-fdebug-prefix-map=$stage_root_physical=/rnode-build\""
fi
extra_flags+=" $path_map_flags"

compile_args=(
  compile
  --clean
  --fqbn "$fqbn"
  --build-path "$stage_root/build"
  --output-dir "$output_dir"
  --build-property "build.partitions=no_ota"
  --build-property "upload.maximum_size=2097152"
  --build-property "compiler.c.extra_flags=$path_map_flags"
  --build-property "compiler.cpp.extra_flags=$extra_flags"
  --build-property "compiler.S.extra_flags=$path_map_flags"
)
if [[ -n ${ARDUINO_CLI_LIBRARY_DIR:-} ]]; then
  compile_args+=(--libraries "$ARDUINO_CLI_LIBRARY_DIR")
fi
compile_args+=("$sketch_dir")

"${cli[@]}" "${compile_args[@]}"

if [[ $action == flash ]]; then
  "${cli[@]}" upload \
    --port "$port" \
    --fqbn "$fqbn" \
    --input-dir "$output_dir" \
    "$sketch_dir"

  firmware_image="$output_dir/RNode_Firmware.ino.bin"
  firmware_hash=$("${PYTHON:-python3}" "$repo_root/Tools/esp_image_hash.py" "$firmware_image")
  rnodeconf=${RNODECONF:-rnodeconf}
  if ! command -v "$rnodeconf" >/dev/null 2>&1; then
    echo "The firmware was flashed, but its integrity hash was not provisioned." >&2
    echo "Install rnodeconf or set RNODECONF=/path/to/rnodeconf, then run:" >&2
    echo "  rnodeconf --firmware-hash $firmware_hash $port" >&2
    exit 1
  fi

  echo "Waiting for the runtime serial port before provisioning the firmware hash..."
  for _ in {1..20}; do
    [[ -e $runtime_port ]] && break
    sleep 0.5
  done
  if [[ ! -e $runtime_port ]]; then
    echo "Runtime serial port did not appear: $runtime_port" >&2
    echo "Provision the hash after rediscovering the port:" >&2
    echo "  rnodeconf --firmware-hash $firmware_hash SERIAL_PORT" >&2
    exit 1
  fi
  sleep 1
  "$rnodeconf" --firmware-hash "$firmware_hash" "$runtime_port"
  echo "Firmware hash provisioned: $firmware_hash"
fi
