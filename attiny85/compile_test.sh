#!/bin/bash
#
# ATTiny85 Compile Test Script
# Tests compilation for internal 8MHz configuration
#

set -e

SKETCH_DIR="$(cd "$(dirname "$0")" && pwd)"
SKETCH_NAME="attiny85.ino"
# Use specific FQBN for ATtiny85 with 8MHz internal clock
BOARD_FQBN="attiny:avr:ATtinyX5:cpu=attiny85,clock=internal8"
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

main() {
    print_header

    echo -e "${YELLOW}Testing 8MHz Internal Oscillator configuration...${NC}"
    
    # Create build dir if not exists
    mkdir -p "${BUILD_DIR}"

    # Compile
    if arduino-cli compile \
        --fqbn "${BOARD_FQBN}" \
        --build-path "${BUILD_DIR}" \
        "${SKETCH_DIR}" > /tmp/compile_output.log 2>&1; then
        
        # Check if hex file was created
        if [ -f "${BUILD_DIR}/${SKETCH_NAME}.hex" ]; then
            echo -e "${GREEN}✓ Compilation successful${NC}"
            
            # Show memory usage
            if grep -q "Sketch uses\|Global variables use" /tmp/compile_output.log; then
                echo "  Memory usage:"
                grep "Sketch uses\|Global variables use" /tmp/compile_output.log | head -2 | sed 's/^/    /'
            fi
            
            echo ""
            echo "Current hex file: ${BUILD_DIR}/${SKETCH_NAME}.hex"
            ls -lh "${BUILD_DIR}/${SKETCH_NAME}.hex"
            exit 0
        else
            echo -e "${RED}✗ Hex file not found${NC}"
            exit 1
        fi
    else
        echo -e "${RED}✗ Compilation failed${NC}"
        echo "  Error output:"
        cat /tmp/compile_output.log | sed 's/^/    /'
        exit 1
    fi
}

main "$@"
