#!/bin/bash
#
# Build all ATtiny85 firmware variants: v2 and v3, each at 8 MHz and 16 MHz.
# v2/v3 are Arduino-core sketches, built with arduino-cli (attiny core) so they
# run the full Arduino init()/analogWrite() environment that reliably drives
# the motor driver. Output: hex/ dir.
#
#   v2_8mhz.hex     v2_16mhz.hex     (Arduino core, integer PI)
#   v3_8mhz.hex     v3_16mhz.hex     (Arduino core + failsafe/anti-windup)
#
# Fuses: 8 MHz -> lfuse 0xE2 (factory default, no PLL) -- RECOMMENDED
#        16 MHz PLL -> lfuse 0xE1 (requires PLL lock; use only if 8 MHz fails)
#   avrdude -c arduino -p attiny85 -P /dev/ttyACM0 -b 19200 -U lfuse:w:0xE2:m  # 8 MHz
#   avrdude -c arduino -p attiny85 -P /dev/ttyACM0 -b 19200 -U lfuse:w:0xE1:m  # 16 MHz

set -e
cd "$(dirname "$0")"

mkdir -p hex
BUILD_DIR="$(mktemp -d)"
trap 'rm -rf "$BUILD_DIR"' EXIT

build() {
    local sketch="$1" tag="$2" clock="$3"
    arduino-cli compile --fqbn "attiny:avr:ATtinyX5:cpu=attiny85,clock=$clock" \
        --build-path "$BUILD_DIR/$tag" "$sketch" >/dev/null 2>&1
    cp "$BUILD_DIR/$tag/$(basename "$sketch").ino.hex" "hex/$tag.hex"
    local size
    size=$(stat -c%s "hex/$tag.hex")
    echo "  built hex/$tag.hex  ($size B, clock=$clock)"
}

echo "Building all variants (Arduino core)..."
build v2 v2_8mhz internal8
build v2 v2_16mhz internal16
build v3 v3_8mhz internal8
build v3 v3_16mhz internal16
echo "Done."
