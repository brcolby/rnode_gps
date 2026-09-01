#!/usr/bin/env bash

set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
test_binary=$(mktemp "${TMPDIR:-/tmp}/rnode-gps-protocol.XXXXXX")

cleanup() {
  rm -f -- "$test_binary"
}
trap cleanup EXIT

"${CXX:-c++}" -std=c++11 -Wall -Wextra -Werror \
  "$repo_root/Tests/telemetry_protocol_test.cpp" -o "$test_binary"
"$test_binary"
