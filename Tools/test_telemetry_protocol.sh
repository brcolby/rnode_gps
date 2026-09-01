#!/usr/bin/env bash

set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
protocol_binary=$(mktemp "${TMPDIR:-/tmp}/rnode-gps-protocol.XXXXXX")
state_binary=$(mktemp "${TMPDIR:-/tmp}/rnode-gps-state.XXXXXX")

cleanup() {
  rm -f -- "$protocol_binary" "$state_binary"
}
trap cleanup EXIT

"${CXX:-c++}" -std=c++11 -Wall -Wextra -Werror \
  "$repo_root/Tests/telemetry_protocol_test.cpp" -o "$protocol_binary"
"$protocol_binary"
"${CXX:-c++}" -std=c++11 -Wall -Wextra -Werror \
  "$repo_root/Tests/telemetry_state_test.cpp" -o "$state_binary"
"$state_binary"
