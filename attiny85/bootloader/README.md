# ATtiny85 Bootloader

This directory contains the bootloader files for ATtiny85 serial programming.

## Optiboot Bootloader

The `optiboot_attiny85.hex` file contains Optiboot bootloader adapted for ATtiny85:

- **Protocol**: Arduino serial
- **Baud Rate**: 9600
- **Flash Size**: ~512 bytes
- **Fuses**:
  - Low fuse: 0xE2 (8MHz internal oscillator)
  - High fuse: 0xDE (512-word bootloader)
  - Extended fuse: 0xFE (self-programming enabled)

## Usage

Use the `burn_bootloader.sh` script to install this bootloader:

```bash
./burn_bootloader.sh [port]
```

After installation, you can program the ATtiny85 via USB to TTL:

```bash
./flash.sh serial /dev/ttyUSB0
```

## Manual Installation

If you prefer manual installation:

1. Set fuses:
   ```bash
   avrdude -p attiny85 -c arduinoisp -P /dev/ttyUSB0 -b 19200 \
     -U lfuse:w:0xE2:m -U hfuse:w:0xDE:m -U efuse:w:0xFE:m
   ```

2. Burn bootloader:
   ```bash
   avrdude -p attiny85 -c arduinoisp -P /dev/ttyUSB0 -b 19200 \
     -U flash:w:bootloader/optiboot_attiny85.hex:i
   ```

## Notes

- The bootloader reserves the last 512 bytes of flash memory
- Serial programming requires manual reset (no auto-reset)
- Timeout is 5 seconds after programming begins
