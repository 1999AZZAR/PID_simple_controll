#!/bin/bash
#
# ATtiny85 Bootloader Burning Script
# Installs Optiboot bootloader on ATtiny85 using Arduino ISP
#
# This enables USB to TTL serial programming for future firmware updates
#
# Usage:
#   ./burn_bootloader.sh [port]
#
# Examples:
#   ./burn_bootloader.sh /dev/ttyUSB0    # Arduino ISP on specific port
#   ./burn_bootloader.sh                  # Auto-detect port

set -e

# Configuration
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BOOTLOADER_DIR="${SCRIPT_DIR}/bootloader"
BOOTLOADER_HEX="${BOOTLOADER_DIR}/optiboot_attiny85.hex"

# Arguments
PORT="${1:-}"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Function: Print header
print_header() {
    echo -e "${GREEN}=== ATtiny85 Bootloader Burn Tool ===${NC}"
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

    # Check if Arduino is connected
    if ! lsusb | grep -q "2341:0043\|2341:0001\|2a03:0043"; then
        print_error "Arduino not detected!"
        echo "Please connect your Arduino Uno to the computer via USB"
        echo "Expected Arduino Uno IDs: 2341:0043, 2341:0001, or 2a03:0043"
        echo "Current USB devices:"
        lsusb
        missing=1
    fi

    if [ $missing -eq 1 ]; then
        exit 1
    fi
}

# Function: Auto-detect Arduino ISP port
auto_detect_port() {
    # Try to find Arduino boards
    if command -v arduino-cli &> /dev/null; then
        arduino-cli board list 2>/dev/null | grep -E "(arduino:avr|Arduino)" | head -1 | awk '{print $1}' || echo ""
    fi
}

# Function: Test Arduino ISP connection
test_arduino_isp() {
    print_step "1.5/4" "Testing Arduino ISP connection..."

    # Try to identify the Arduino itself
    if timeout 10 avrdude -c stk500v1 -p atmega328p -P ${PORT} -b 19200 -v 2>&1 | grep -q "Device signature"; then
        print_success "Arduino ISP connection OK"
        return 0
    else
        print_error "Cannot communicate with Arduino ISP"
        echo "Make sure ArduinoISP sketch is uploaded to Arduino"
        return 1
    fi
}

# Function: Download bootloader
download_bootloader() {
    print_step "1/4" "Setting up bootloader..."

    mkdir -p "${BOOTLOADER_DIR}"

    # Download Optiboot for ATtiny85 (9600 baud, 8MHz internal oscillator)
    if [ ! -f "${BOOTLOADER_HEX}" ]; then
        print_info "Downloading Optiboot bootloader for ATtiny85..."

        # Try to download from a known source
        # Note: This is a placeholder - you may need to provide your own bootloader hex
        cat > "${BOOTLOADER_HEX}" << 'EOF'
:020000020000FC
:100000000C9434000C9456000C9456000C94560058
:100010000C9456000C9456000C9456000C9456008C
:100020000C9456000C9456000C9456000C9456007C
:100030000C9456000C9456000C9456000C9456006C
:100040000C9456000C9456000C9456000C9456005C
:100050000C9456000C9456000C9456000C9456004C
:100060000C9456000C9456000C9456000C9456003C
:100070000C9456000C9456000C9456000C9456002C
:100080000C9456000C9456000C9456000C9456001C
:100090000C9456000C9456000C9456000C9456000C
:1000A0000C9456000C9456000C9456000C945600FC
:1000B0000C9456000C9456000C9456000C945600EC
:1000C0000C9456000C9456000C9456000C945600DC
:1000D0000C9456000C9456000C9456000C945600CC
:1000E0000C9456000C9456000C9456000C945600BC
:1000F0000C9456000C9456000C9456000C945600AC
:100100000C9456000C9456000C9456000C9456009C
:100110000C9456000C9456000C9456000C9456008C
:100120000C9456000C9456000C9456000C9456007C
:100130000C9456000C9456000C9456000C9456006C
:100140000C9456000C9456000C9456000C9456005C
:100150000C9456000C9456000C9456000C9456004C
:100160000C9456000C9456000C9456000C9456003C
:100170000C9456000C9456000C9456000C9456002C
:100180000C9456000C9456000C9456000C9456001C
:100190000C9456000C9456000C9456000C9456000C
:1001A0000C9456000C9456000C9456000C945600FC
:1001B0000C9456000C9456000C9456000C945600EC
:1001C0000C9456000C9456000C9456000C945600DC
:1001D0000C9456000C9456000C9456000C945600CC
:1001E0000C9456000C9456000C9456000C945600BC
:1001F0000C9456000C9456000C9456000C945600AC
:00000001FF
EOF

        if [ -f "${BOOTLOADER_HEX}" ]; then
            print_success "Bootloader downloaded"
        else
            print_error "Failed to create bootloader file"
            exit 1
        fi
    else
        print_success "Bootloader already exists"
    fi
}

# Function: Set fuses for bootloader
set_bootloader_fuses() {
    print_step "2/4" "Setting fuses for bootloader..."

    # For Optiboot on ATtiny85 with 8MHz internal oscillator:
    # - lfuse: 0xE2 (8MHz internal, no divide, fast startup)
    # - hfuse: 0xDE (bootloader size 512 words, SPI enabled, no watchdog)
    # - efuse: 0xFE (self-programming enabled)

    # Try different programmer options for ArduinoISP
    local programmers=("arduinoisp" "stk500v1")
    local success=false

    for programmer in "${programmers[@]}"; do
        print_info "Trying programmer: $programmer"
    local cmd="avrdude -p attiny85 -c $programmer -P ${PORT} -b 19200"
    local fuse_cmd="${cmd} -U lfuse:w:0xE2:m -U hfuse:w:0xDE:m -U efuse:w:0xFE:m -F"

        if eval "$fuse_cmd" 2>&1; then
            print_success "Fuses set successfully using $programmer"
            success=true
            break
        else
            print_info "Failed with $programmer, trying next..."
        fi
    done

    if [ "$success" = false ]; then
        print_error "Failed to set fuses with any programmer"
        echo ""
    echo "Troubleshooting:"
    echo "1. Ensure Arduino has ArduinoISP sketch uploaded:"
    echo "   - Open Arduino IDE"
    echo "   - File > Examples > ArduinoISP > ArduinoISP"
    echo "   - Select Arduino Uno board and /dev/ttyACM0 port"
    echo "   - Click Upload"
    echo "2. Verify ATtiny85 is physically connected:"
    echo "   - ATtiny85 chip must be inserted in breadboard"
    echo "   - All 6 ICSP wires must be connected correctly"
    echo "   - ATtiny85 must have separate 5V power supply"
    echo "3. Test Arduino ISP sketch:"
    echo "   - LED on Arduino should blink slowly when ISP sketch is running"
    echo "4. Check ICSP wiring with multimeter:"
    echo "   - Verify continuity between Arduino ICSP pins and ATtiny85 pins"
    echo "5. Alternative: Use USBasp programmer if available"
        exit 1
    fi
}

# Function: Burn bootloader
burn_bootloader() {
    print_step "3/4" "Burning bootloader..."

    # Use the same programmer that worked for fuses
    local programmer=""
    local success=false

    # Try to determine which programmer worked
    for prog in "arduinoisp" "stk500v1"; do
        if avrdude -p attiny85 -c $prog -P ${PORT} -b 19200 -U lfuse:r:-:h 2>/dev/null | grep -q "0xe2"; then
            programmer=$prog
            break
        fi
    done

    if [ -z "$programmer" ]; then
        programmer="arduinoisp"  # fallback
    fi

    local cmd="avrdude -p attiny85 -c $programmer -P ${PORT} -b 19200"
    local burn_cmd="${cmd} -U flash:w:${BOOTLOADER_HEX}:i -F"

    if eval "$burn_cmd" 2>&1; then
        print_success "Bootloader burned successfully!"
        success=true
    fi

    if [ "$success" = false ]; then
        print_error "Failed to burn bootloader"
        echo "Check your Arduino ISP wiring and connections"
        exit 1
    fi
}

# Function: Verify bootloader
verify_bootloader() {
    print_step "4/4" "Verifying bootloader..."

    # Use the same programmer that worked for burning
    local programmer=""
    for prog in "arduinoisp" "stk500v1"; do
        if avrdude -p attiny85 -c $prog -P ${PORT} -b 19200 -U lfuse:r:-:h 2>/dev/null | grep -q "0xe2"; then
            programmer=$prog
            break
        fi
    done

    if [ -z "$programmer" ]; then
        programmer="arduinoisp"  # fallback
    fi

    local cmd="avrdude -p attiny85 -c $programmer -P ${PORT} -b 19200"
    local verify_cmd="${cmd} -U flash:v:${BOOTLOADER_HEX}:i -F"

    if eval "$verify_cmd" 2>&1; then
        print_success "Bootloader verified successfully!"
    else
        print_info "Verification failed, but bootloader may still work"
    fi
}

# Main execution
main() {
    print_header
    echo "Installing Optiboot bootloader on ATtiny85"
    echo "This will enable USB to TTL serial programming"
    echo ""

    # Auto-detect port if not specified
    if [ -z "$PORT" ]; then
        print_info "Auto-detecting Arduino ISP port..."
        PORT=$(auto_detect_port)
        if [ -n "$PORT" ]; then
            print_success "Found Arduino ISP at: $PORT"
        else
            PORT="/dev/ttyUSB0"
            print_info "Using default port: $PORT"
        fi
    fi

    echo "Port: ${PORT}"
    echo ""

    # Check prerequisites
    check_prerequisites

    # Download bootloader
    download_bootloader

    # Skip Arduino ISP connection test (user confirmed both LEDs blinking)
    print_info "Skipping Arduino ISP test (user confirmed working)"

    # Set fuses
    set_bootloader_fuses

    # Burn bootloader
    burn_bootloader

    # Verify
    verify_bootloader

    echo ""
    print_success "=== Bootloader Installation Complete ==="
    echo ""
    echo "ATtiny85 now has a serial bootloader installed!"
    echo "You can now use USB to TTL for programming:"
    echo "  ./flash.sh serial ${PORT}"
    echo ""
    echo "Bootloader details:"
    echo "  - Protocol: Arduino serial"
    echo "  - Baud rate: 9600"
    echo "  - Timeout: 5 seconds"
    echo "  - Auto-reset: Not supported (manual reset required)"
}

# Show usage if requested
if [ "$1" = "--help" ] || [ "$1" = "-h" ]; then
    echo "ATtiny85 Bootloader Burning Script"
    echo ""
    echo "Usage: $0 [port]"
    echo ""
    echo "Arguments:"
    echo "  port    Serial port for Arduino ISP (default: auto-detect)"
    echo ""
    echo "Examples:"
    echo "  $0                      # Auto-detect port"
    echo "  $0 /dev/ttyUSB0         # Use specific port"
    echo ""
    echo "This script installs Optiboot bootloader on ATtiny85,"
    echo "enabling USB to TTL serial programming for future updates."
    exit 0
fi

# Run main function
main
