#!/usr/bin/env bash
# Syntax-check every sketch with a host g++, using the stub headers.
#
# There is no arduino-cli on the machine this was written for, and several
# sketches here had never been compiled by anything. This is the cheap 95%:
# it will not tell you the flash figure, but it will not let you take an
# undeclared identifier to the bench either.
#
#   ./tools/syntax_check.sh          one build of each sketch, including archive/
#   ./tools/syntax_check.sh --matrix plus every actuator_v1 and actuator_v2
#                                    config permutation
#
# Rescued from a branch (claude/imu-fullstack-grk93v) that added IMU support
# never merged here -- the --matrix section below is rewritten against the
# config switches actuator_v1 and actuator_v2 actually have on this branch,
# not the IMU/sensor knobs that commit's actuator_v1 carried.
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

check_dir() {  # check_dir <label> <dir>
  local label="$1" dir="$2"
  local ino
  ino=$(ls "$dir"*.ino 2>/dev/null | head -1) || return
  [ -n "$ino" ] || return
  # Sketches that do not include Arduino.h rely on the IDE prepending it.
  if grep -q '#include *<Arduino.h>' "$ino"; then
    check "$label" "$dir" "$ino"
  else
    check "$label" "$dir" "$ino" -include Arduino.h
  fi
}

echo "sketches"
for d in "$ROOT"/*/; do
  name="$(basename "$d")"
  [ "$name" = "archive" ] && continue
  check_dir "$name" "$d"
done

echo
echo "archive/"
for d in "$ROOT"/archive/*/; do
  check_dir "archive/$(basename "$d")" "$d"
done

if [ "${1:-}" = "--matrix" ]; then
  tmp=$(mktemp -d); trap 'rm -rf "$tmp"' EXIT

  echo
  echo "actuator_v1 config matrix"
  n=0
  for drv in DRV_L298N DRV_HW039; do
   for eep in 1 0; do
    for lcd in 0 1; do
      rm -rf "$tmp/b"; cp -r "$ROOT/archive/actuator_v1" "$tmp/b"
      sed -i "s/^#define MOTOR_DRIVER .*/#define MOTOR_DRIVER  $drv/; \
              s/^#define USE_EEPROM .*/#define USE_EEPROM $eep/; \
              s/^#define USE_LCD .*/#define USE_LCD $lcd/" "$tmp/b/Config.h"
      out=$(cd "$tmp/b" && "$CXX" "${FLAGS[@]}" -I. actuator_v1.ino 2>&1)
      n=$((n+1))
      if [ -z "$out" ]; then pass=$((pass+1)); else
        fail=$((fail+1))
        echo "  FAIL drv=$drv eep=$eep lcd=$lcd"
        echo "$out" | head -8
      fi
    done; done; done
  echo "  $n permutations checked"

  echo
  echo "actuator_v2 config matrix"
  n=0
  for drv in DRV_L298N DRV_HW039 DRV_G2; do
   for btn in 1 0; do
    for buz in 1 0; do
     for kill in 1 0; do
      rm -rf "$tmp/b"; cp -r "$ROOT/actuator_v2" "$tmp/b"
      sed -i "s/^#define MOTOR_DRIVER .*/#define MOTOR_DRIVER  $drv/; \
              s/^#define USE_BUTTONS .*/#define USE_BUTTONS $btn/; \
              s/^#define USE_BUZZER .*/#define USE_BUZZER  $buz/; \
              s/^#define USE_KILL .*/#define USE_KILL     $kill/" "$tmp/b/Config.h"
      out=$(cd "$tmp/b" && "$CXX" "${FLAGS[@]}" -I. actuator_v2.ino 2>&1)
      n=$((n+1))
      if [ -z "$out" ]; then pass=$((pass+1)); else
        fail=$((fail+1))
        echo "  FAIL drv=$drv btn=$btn buz=$buz kill=$kill"
        echo "$out" | head -8
      fi
     done; done; done; done
  echo "  $n permutations checked"
fi

echo
echo "$pass passed, $fail failed"
[ "$fail" -eq 0 ]
