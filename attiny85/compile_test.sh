#!/bin/bash
#
# ATTiny85 Compile Test Script
# Tests compilation for both internal and external crystal configurations
#

set -e

SKETCH_DIR="$(cd "$(dirname "$0")" && pwd)"
SKETCH_NAME="attiny85.ino"
BOARD_FQBN="attiny:avr:ATtinyX5"
BUILD_DIR="${SKETCH_DIR}/compiled"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_header() {
    echo -e "${BLUE}=== ATTiny85 Compile Test ===${NC}"
}

test_compilation() {
    local config_name="$1"
    echo -e "${YELLOW}Testing $config_name configuration...${NC}"

    # Compile
    if arduino-cli compile \
        --fqbn "${BOARD_FQBN}" \
        --build-path "${BUILD_DIR}" \
        "${SKETCH_DIR}" > /tmp/compile_output.log 2>&1; then

        # Check if hex file was created
        if [ -f "${BUILD_DIR}/${SKETCH_NAME}.hex" ]; then
            echo -e "${GREEN}✓ $config_name compilation successful${NC}"

            # Show memory usage
            if grep -q "Sketch uses\|Global variables use" /tmp/compile_output.log; then
                echo "  Memory usage:"
                grep "Sketch uses\|Global variables use" /tmp/compile_output.log | head -2 | sed 's/^/    /'
            fi
            return 0
        else
            echo -e "${RED}✗ Hex file not found${NC}"
            return 1
        fi
    else
        echo -e "${RED}✗ $config_name compilation failed${NC}"
        echo "  Error output:"
        cat /tmp/compile_output.log | sed 's/^/    /'
        return 1
    fi
}

main() {
    print_header

    echo "Testing both ATTiny85 configurations..."
    echo ""

    # Test internal oscillator (8MHz)
    ./switch_config.sh internal > /dev/null 2>&1
    if test_compilation "8MHz Internal Oscillator"; then
        internal_ok=1
    else
        internal_ok=0
    fi

    echo ""

    # Test external crystal (20MHz)
    ./switch_config.sh external > /dev/null 2>&1
    if test_compilation "20MHz External Crystal"; then
        external_ok=1
    else
        external_ok=0
    fi

    echo ""
    echo -e "${BLUE}=== Results ===${NC}"

    if [ $internal_ok -eq 1 ] && [ $external_ok -eq 1 ]; then
        echo -e "${GREEN}✓ All configurations compiled successfully!${NC}"
        echo ""
        echo "Current hex file: ${BUILD_DIR}/${SKETCH_NAME}.hex"
        ls -lh "${BUILD_DIR}/${SKETCH_NAME}.hex"
        exit 0
    else
        echo -e "${RED}✗ Some configurations failed to compile${NC}"
        exit 1
    fi
}

main "$@"