#!/bin/bash
#
# ATtiny85 Flashing Script - Unified Version
# Compiles and flashes the BLDC PID Controller to ATtiny85
#
# Supports multiple programmers:
#   - Arduino as ISP (arduinoisp)
#   - USBasp (usbasp)
#   - USBtinyISP (usbtiny)
#   - AVR ISP mkII (avrisp2)
#   - QYF0893/Digispark USB boards (auto-detected)
#
# Usage:
#   ./flash.sh [programmer] [port]
#
# Examples:
#   ./flash.sh                          # Auto-detect or use default (arduinoisp)
#   ./flash.sh arduinoisp /dev/ttyUSB0  # Arduino as ISP
#   ./flash.sh usbasp                   # USBasp programmer
#   ./flash.sh qyf0893                  # QYF0893 USB board
#   ./flash.sh digispark                # Digispark USB board
#   ./flash.sh auto                     # Auto-detect USB programmers

set -e

# Configuration
SKETCH_DIR="$(cd "$(dirname "$0")" && pwd)"
SKETCH_NAME="attiny85.ino"
BOARD_FQBN="attiny:avr:ATtinyX5"
BUILD_DIR="${SKETCH_DIR}/compiled"

# Arguments
PROGRAMMER="${1:-}"
PORT="${2:-}"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Function: Print section header
print_header() {
    echo -e "${GREEN}=== ATtiny85 Flash Tool ===${NC}"
}

# Function: Print step
print_step() {
    echo -e "${YELLOW}[$1] $2${NC}"
}

# Function: Print success
print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

# Function: Print error
print_error() {
    echo -e "${RED}✗ $1${NC}"
}

# Function: Print info
print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

# Function: Auto-detect USB programmer
auto_detect_programmer() {
    if lsusb | grep -qi "16c0:05df\|1781:0c9f"; then
        echo "usbasp"
        return 0
    fi
    if lsusb | grep -qi "16c0:05dc\|1781:0c98"; then
        echo "usbtiny"
        return 0
    fi
    return 1
}

# Function: Handle programmer aliases
normalize_programmer() {
    case "${PROGRAMMER,,}" in
        qyf0893|digispark)
            PROGRAMMER="usbasp"
            PORT=""
            print_info "QYF0893/Digispark detected - using USBasp protocol"
            ;;
        auto)
            if detected=$(auto_detect_programmer); then
                PROGRAMMER="$detected"
                PORT=""
                print_info "Auto-detected programmer: $PROGRAMMER"
            else
                print_info "No USB programmer detected, trying common methods..."
                PROGRAMMER="usbasp"
                PORT=""
            fi
            ;;
        "")
            # No programmer specified - try auto-detect, fallback to arduinoisp
            if detected=$(auto_detect_programmer); then
                PROGRAMMER="$detected"
                PORT=""
                print_info "Auto-detected programmer: $PROGRAMMER"
            else
                PROGRAMMER="arduinoisp"
                PORT="${PORT:-/dev/ttyUSB0}"
                print_info "Using default: Arduino as ISP (specify programmer to override)"
            fi
            ;;
    esac
}

# Function: Check prerequisites
check_prerequisites() {
    local missing=0
    
    if ! command -v arduino-cli &> /dev/null; then
        print_error "arduino-cli not found!"
        echo "Install with: curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh"
        missing=1
    fi
    
    if ! command -v avrdude &> /dev/null; then
        print_error "avrdude not found!"
        echo "Install with: sudo apt-get install avrdude"
        missing=1
    fi
    
    if [ $missing -eq 1 ]; then
        exit 1
    fi
}

# Function: Install board support
install_board_support() {
    print_step "1/5" "Checking board support..."
    arduino-cli core update-index 2>/dev/null || true
    if ! arduino-cli core list | grep -q "attiny:avr"; then
        print_info "Installing ATtiny core..."
        arduino-cli core install attiny:avr
    fi
}

# Function: Compile sketch
compile_sketch() {
    print_step "2/5" "Compiling sketch..."
    mkdir -p "${BUILD_DIR}"
    
    if arduino-cli compile \
        --fqbn "${BOARD_FQBN}" \
        --build-path "${BUILD_DIR}" \
        --quiet \
        "${SKETCH_DIR}" 2>&1 | tee /tmp/compile.log; then
        print_success "Compilation successful"
        
        # Show size info
        if grep -q "Sketch uses" /tmp/compile.log; then
            grep "Sketch uses\|Global variables use" /tmp/compile.log | head -2
        fi
    else
        print_error "Compilation failed!"
        echo "Check compilation errors above"
        exit 1
    fi
}

# Function: Find hex file
find_hex_file() {
    local hex_file=""
    
    for possible in \
        "${BUILD_DIR}/${SKETCH_NAME}.hex" \
        "${BUILD_DIR}/${SKETCH_NAME}.with_bootloader.hex" \
        "${BUILD_DIR}/attiny85.hex" \
        "${BUILD_DIR}"/*.hex; do
        if [ -f "$possible" ]; then
            hex_file="$possible"
            break
        fi
    done
    
    if [ -z "$hex_file" ] || [ ! -f "$hex_file" ]; then
        print_error "Hex file not found!"
        echo "Searched in: ${BUILD_DIR}"
        find "${BUILD_DIR}" -name "*.hex" -type f 2>/dev/null || echo "No hex files found"
        exit 1
    fi
    
    echo "$hex_file"
}

# Function: Build avrdude command
build_avrdude_cmd() {
    local cmd="avrdude -p attiny85 -c ${PROGRAMMER}"
    
    # Add port if specified and needed
    if [ -n "$PORT" ] && [[ "$PROGRAMMER" != "usbasp" && "$PROGRAMMER" != "usbtiny" ]]; then
        cmd="${cmd} -P ${PORT}"
    fi
    
    # Set baud rate for ArduinoISP
    if [ "$PROGRAMMER" = "arduinoisp" ]; then
        cmd="${cmd} -b 19200"
    fi
    
    echo "$cmd"
}

# Function: Set fuses
set_fuses() {
    print_step "3/5" "Setting fuses for 8MHz internal oscillator..."
    local cmd=$(build_avrdude_cmd)
    local fuse_cmd="${cmd} -U lfuse:w:0xE2:m -U hfuse:w:0xDF:m -U efuse:w:0xFF:m"
    
    if eval "$fuse_cmd" &>/dev/null; then
        print_success "Fuses set successfully"
    else
        print_info "Fuse setting skipped (may already be set or programmer doesn't support it)"
    fi
}

# Function: Flash program
flash_program() {
    print_step "4/5" "Flashing program..."
    local hex_file="$1"
    local cmd=$(build_avrdude_cmd)
    local flash_cmd="${cmd} -U flash:w:${hex_file}:i"
    
    # For USB programmers, try multiple methods if first fails
    if [[ "$PROGRAMMER" == "usbasp" ]] && ! eval "$flash_cmd" 2>&1; then
        print_info "USBasp failed, trying USBtiny..."
        cmd="avrdude -p attiny85 -c usbtiny"
        flash_cmd="${cmd} -U flash:w:${hex_file}:i"
    fi
    
    if eval "$flash_cmd" 2>&1; then
        print_success "Flashing successful!"
        return 0
    else
        return 1
    fi
}

# Function: Try micronucleus (for bootloader-based boards)
try_micronucleus() {
    local hex_file="$1"
    
    if ! command -v micronucleus &> /dev/null; then
        return 1
    fi
    
    print_info "Trying Micronucleus bootloader..."
    echo "Please plug in the device now (you have 5 seconds)..."
    sleep 5
    
    if micronucleus --run "${hex_file}" 2>&1; then
        print_success "Flashing successful via Micronucleus!"
        return 0
    fi
    
    return 1
}

# Main execution
main() {
    print_header
    echo "Sketch: ${SKETCH_NAME}"
    echo "Board: ${BOARD_FQBN}"
    echo ""
    
    # Normalize programmer name
    normalize_programmer
    
    echo "Programmer: ${PROGRAMMER}"
    [ -n "$PORT" ] && echo "Port: ${PORT}"
    echo ""
    
    # Check prerequisites
    check_prerequisites
    
    # Install board support
    install_board_support
    
    # Compile
    compile_sketch
    
    # Find hex file
    HEX_FILE=$(find_hex_file)
    print_success "Found hex file: ${HEX_FILE}"
    echo ""
    
    # Set fuses (skip for bootloader-based boards)
    if [[ "$PROGRAMMER" != "micronucleus" ]]; then
        set_fuses
    fi
    
    # Flash
    print_step "5/5" "Flashing program..."
    if flash_program "$HEX_FILE"; then
        echo ""
        print_success "=== Flash Complete ==="
        echo "ATtiny85 is now programmed with BLDC PID Controller"
        exit 0
    fi
    
    # If USB programmer failed, try micronucleus
    if [[ "$PROGRAMMER" == "usbasp" ]] || [[ "$PROGRAMMER" == "usbtiny" ]]; then
        if try_micronucleus "$HEX_FILE"; then
            exit 0
        fi
    fi
    
    # All methods failed
    echo ""
    print_error "Flashing failed!"
    echo ""
    echo "Troubleshooting:"
    echo "1. Check programmer connection"
    echo "2. Verify port: ${PORT:-N/A}"
    echo "3. Try different programmer: ./flash.sh usbasp"
    echo "4. Try auto-detection: ./flash.sh auto"
    echo "5. Check permissions: sudo usermod -a -G dialout \$USER"
    echo "6. See FLASHING_GUIDE.md for detailed instructions"
    exit 1
}

# Run main function
main
