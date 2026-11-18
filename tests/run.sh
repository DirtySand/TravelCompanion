#!/bin/sh
# Host tests for the hardware-independent modules: sh tests/run.sh
set -e
cd "$(dirname "$0")/.."
OUT="${TMPDIR:-/tmp}/tc-tests"
mkdir -p "$OUT"
CC="${CC:-cc}"
CFLAGS="-std=c11 -Wall -Wextra -Werror"
INC="-Idrivers/bme680 -Idrivers/ds3231 -Idrivers/neo6m -Isrc/time -Isrc/motion -Isrc/nav"

build_and_run() {
  name="$1"
  shift
  $CC $CFLAGS $INC -o "$OUT/$name" "tests/$name.c" "$@" -lm
  "$OUT/$name"
}

build_and_run test_bme680 drivers/bme680/bme680.c drivers/bme680/bme680_compensation.c
build_and_run test_rtc drivers/ds3231/ds3231.c src/time/time_sync.c
build_and_run test_steps src/motion/steps.c
build_and_run test_datetime src/time/datetime_utils.c src/time/timezone.c
build_and_run test_fusion src/nav/fusion.c src/nav/geo.c
echo "all host tests passed"
