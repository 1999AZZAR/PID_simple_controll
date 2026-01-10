#!/bin/bash
#
# ATTiny85 Configuration Switcher
# Easily switch between internal oscillator (8MHz) and external crystal (20MHz) versions
#
# Usage:
#   ./switch_config.sh internal    # Switch to 8MHz internal oscillator
#   ./switch_config.sh external    # Switch to 20MHz external crystal
#   ./switch_config.sh status      # Show current configuration

COMMON_CONFIG_FILE="config_common.h"
BACKUP_FILE="config_common.h.backup"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_header() {
    echo -e "${BLUE}=== ATTiny85 Configuration Switcher ===${NC}"
}

show_usage() {
    echo "Usage: $0 {internal|external|status}"
    echo ""
    echo "Commands:"
    echo "  internal  - Switch to 8MHz internal oscillator version"
    echo "  external  - Switch to 20MHz external crystal version"
    echo "  status    - Show current configuration"
    echo ""
    echo "Hardware requirements for external crystal:"
    echo "  - 16MHz crystal between pins 2-3 (PB3-PB4)"
    echo "  - Two 22pF capacitors from crystal to ground"
    echo "  - Correct fuse settings (set by programmer)"
}

get_current_config() {
    if grep -q '^#define USE_EXTERNAL_CRYSTAL' "$COMMON_CONFIG_FILE"; then
        echo "external"
    elif grep -q '^#define USE_INTERNAL_OSCILLATOR' "$COMMON_CONFIG_FILE"; then
        echo "internal"
    else
        echo "unknown"
    fi
}

show_status() {
    local current=$(get_current_config)

    echo "Current configuration: "
    case $current in
        "internal")
            echo -e "${GREEN}✓ 8MHz Internal Oscillator${NC}"
            echo "  - No external components needed"
            echo "  - Control loop: 160Hz"
            echo "  - PWM frequency: ~1kHz"
            ;;
        "external")
            echo -e "${GREEN}✓ 20MHz External Crystal${NC}"
            echo "  - Requires 16MHz crystal + 2x 22pF capacitors"
            echo "  - Control loop: 200Hz (25% faster)"
            echo "  - PWM frequency: ~1.2kHz"
            ;;
        *)
            echo -e "${RED}✗ Unknown configuration${NC}"
            echo "  Please check $CONFIG_FILE"
            ;;
    esac
}

switch_config() {
    local target="$1"

    # Backup current file
    cp "$COMMON_CONFIG_FILE" "$BACKUP_FILE"

    # Comment out both defines first
    sed -i 's|^#define USE_EXTERNAL_CRYSTAL|//#define USE_EXTERNAL_CRYSTAL|' "$COMMON_CONFIG_FILE"
    sed -i 's|^#define USE_INTERNAL_OSCILLATOR|//#define USE_INTERNAL_OSCILLATOR|' "$COMMON_CONFIG_FILE"

    # Uncomment the target define
    case $target in
        "internal")
            sed -i 's|^//#define USE_INTERNAL_OSCILLATOR|#define USE_INTERNAL_OSCILLATOR|' "$COMMON_CONFIG_FILE"
            ;;
        "external")
            sed -i 's|^//#define USE_EXTERNAL_CRYSTAL|#define USE_EXTERNAL_CRYSTAL|' "$COMMON_CONFIG_FILE"
            ;;
        *)
            echo -e "${RED}Error: Invalid target '$target'${NC}"
            show_usage
            # Restore backup
            mv "$BACKUP_FILE" "$COMMON_CONFIG_FILE"
            exit 1
            ;;
    esac

    echo -e "${GREEN}✓ Successfully switched to $target configuration${NC}"
    echo "  Backup saved as $BACKUP_FILE"
    show_status
}

# Main logic
print_header

case "${1:-status}" in
    "internal")
        switch_config "internal"
        ;;
    "external")
        switch_config "external"
        ;;
    "status")
        show_status
        ;;
    *)
        echo -e "${RED}Error: Unknown command '$1'${NC}"
        echo ""
        show_usage
        exit 1
        ;;
esac