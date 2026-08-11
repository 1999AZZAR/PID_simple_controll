#!/bin/bash
#
# Flash the correct firmware variant for the connected ATtiny85 based on its
# fused clock. Auto-detects 8 MHz (lfuse 0xE2) vs 16 MHz (lfuse 0xE1) and picks
# the matching hex from hex/.
#
#   ./flash.sh v3          # flash v3 (auto clock match)
#   ./flash.sh v2          # flash v2
#   ./flash.sh --read      # just report signature + fuses
#
# Dependencies: avrdude, hex/ built via build_all.sh

set -e
cd "$(dirname "$0")"

PORT="${PORT:-/dev/ttyACM0}"
PROG="${PROG:-arduino}"
BAUD=19200
VER="${1:-v3}"

sig_fuses() {
    avrdude -c "$PROG" -p attiny85 -P "$PORT" -b $BAUD \
        -U signature:r:/tmp/t85_sig.txt:h \
        -U lfuse:r:/tmp/t85_lfuse.txt:h 2>&1
}

echo "Reading target chip ($PORT)..."
sig_fuses | grep -iE "signature|done|error" || true

SIG=$(cat /tmp/t85_sig.txt 2>/dev/null | tr -d '\r')
LFUSE=$(cat /tmp/t85_lfuse.txt 2>/dev/null | tr -d '\r')

if [ -z "$SIG" ]; then
    echo "ERROR: could not read signature. Check wiring/programmer." >&2
    exit 1
fi
echo "Signature: $SIG"

case "$LFUSE" in
    0xE2|0xe2) CLOCK="8mhz" ;;
    0xE1|0xe1) CLOCK="16mhz" ;;
    *)
        echo "ERROR: unexpected lfuse 0x$LFUSE (expected 0xE2=8MHz or 0xE1=16MHz)." >&2
        exit 1
        ;;
esac
echo "lfuse:     $LFUSE ($CLOCK)"

HEX="hex/${VER}_${CLOCK}.hex"
if [ ! -f "$HEX" ]; then
    echo "ERROR: $HEX not found. Run ./build_all.sh first." >&2
    exit 1
fi
echo "Flashing $HEX ..."
avrdude -c "$PROG" -p attiny85 -P "$PORT" -b $BAUD -U flash:w:"$HEX":i 2>&1 \
    | grep -iE "flash|verify|done|error" || true
echo "Done."
