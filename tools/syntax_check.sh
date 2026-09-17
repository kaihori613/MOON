#!/usr/bin/env bash
# Syntax-check every sketch with a host g++, using the stub headers.
#
# There is no arduino-cli on the machine this was written for, and several
# sketches here had never been compiled by anything. This is the cheap 95%:
# it will not tell you the flash figure, but it will not let you take an
# undeclared identifier to the bench either.
#
#   ./tools/syntax_check.sh          one build of each sketch
#   ./tools/syntax_check.sh --matrix plus every actuator_v1 config permutation
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
STUB="$ROOT/tools/arduino_stub"
CXX="${CXX:-g++}"
FLAGS=(-fsyntax-only -x c++ -std=gnu++17 -Wall -Wextra
       -Wno-unused-parameter -Wno-unused-variable -I"$STUB")

pass=0; fail=0

check() {  # check <label> <dir> <file> [extra flags...]
  local label="$1" dir="$2" file="$3"; shift 3
  local out
  out=$(cd "$dir" && "$CXX" "${FLAGS[@]}" -I. "$@" "$file" 2>&1)
  if [ -z "$out" ]; then
    printf '  %-46s clean\n' "$label"; pass=$((pass+1))
  else
    printf '  %-46s FAIL\n' "$label"; echo "$out" | head -12; fail=$((fail+1))
  fi
}

echo "sketches"
for d in "$ROOT"/*/; do
  ino=$(ls "$d"*.ino 2>/dev/null | head -1) || continue
  [ -n "$ino" ] || continue
  # Sketches that do not include Arduino.h rely on the IDE prepending it.
  if grep -q '#include *<Arduino.h>' "$ino"; then
    check "$(basename "$d")" "$d" "$ino"
  else
    check "$(basename "$d")" "$d" "$ino" -include Arduino.h
  fi
done

if [ "${1:-}" = "--matrix" ]; then
  echo
  echo "actuator_v1 config matrix"
  tmp=$(mktemp -d); trap 'rm -rf "$tmp"' EXIT
  for drv in DRV_L298N DRV_HW039; do
   for imu in 1 0; do
    for sens in IMU_FXOS8700 IMU_MPU9150; do
     for lcd in 0 1; do
      for buz in 1 0; do
       for eep in 1 0; do
        rm -rf "$tmp/b"; cp -r "$ROOT/actuator_v1" "$tmp/b"
        sed -i "s/^#define MOTOR_DRIVER .*/#define MOTOR_DRIVER  $drv/; \
                s/^#define USE_IMU .*/#define USE_IMU     $imu/; \
                s/^#define IMU_SENSOR .*/#define IMU_SENSOR  $sens/; \
                s/^#define USE_LCD .*/#define USE_LCD $lcd/; \
                s/^#define USE_BUZZER .*/#define USE_BUZZER $buz/; \
                s/^#define USE_EEPROM .*/#define USE_EEPROM $eep/" "$tmp/b/Config.h"
        out=$(cd "$tmp/b" && "$CXX" "${FLAGS[@]}" -I. actuator_v1.ino 2>&1)
        if [ -z "$out" ]; then pass=$((pass+1)); else
          fail=$((fail+1))
          echo "  FAIL drv=$drv imu=$imu sens=$sens lcd=$lcd buz=$buz eep=$eep"
          echo "$out" | head -8
        fi
       done; done; done; done; done; done
  echo "  64 permutations checked"
fi

echo
echo "$pass passed, $fail failed"
[ "$fail" -eq 0 ]
