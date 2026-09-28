#!/usr/bin/env bash

set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cli=("${ARDUINO_CLI:-arduino-cli}")
if [[ -n ${ARDUINO_CLI_CONFIG_DIR:-} ]]; then
  cli+=(--config-dir "$ARDUINO_CLI_CONFIG_DIR")
fi
cli+=(--config-file "${ARDUINO_CLI_CONFIG:-$repo_root/arduino-cli.yaml}")

echo "Arduino CLI"
"${cli[@]}" version
echo
echo "Platforms"
"${cli[@]}" core list
echo
echo "Libraries"
"${cli[@]}" lib list
echo
printf 'SHA-256\tbytes\tartifact\n'

find \
  "$repo_root/build/stock-tbeam-supreme" \
  "$repo_root/build/telemetry-usb" \
  "$repo_root/build/telemetry-uart" \
  -maxdepth 1 -type f \( -name '*.bin' -o -name '*.elf' \) -print |
  LC_ALL=C sort |
  while IFS= read -r artifact; do
    digest=$(shasum -a 256 "$artifact" | awk '{print $1}')
    bytes=$(wc -c < "$artifact" | tr -d ' ')
    printf '%s\t%s\t%s\n' "$digest" "$bytes" "${artifact#"$repo_root"/}"
  done
