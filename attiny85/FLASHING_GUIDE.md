# ATtiny85 Flashing Guide

Complete guide for flashing the BLDC PID Controller firmware to ATtiny85 microcontroller.

## Table of Contents

- [Quick Start](#quick-start)
- [Prerequisites](#prerequisites)
- [Supported Programmers](#supported-programmers)
- [Detailed Instructions](#detailed-instructions)
- [USB to TTL Serial Programming](#usb-to-ttl-serial-programming)
- [QYF0893 USB Development Board](#qyf0893-usb-development-board)
- [Hardware Setup](#hardware-setup)
- [Troubleshooting](#troubleshooting)
- [Manual Flashing](#manual-flashing)

## Quick Start

```bash
cd /home/azzar/project/PIDs/PID_simple_controll/attiny85

# Auto-detect USB programmer (recommended for QYF0893)
./flash.sh auto

# Or specify programmer
./flash.sh usbasp              # USBasp/USB-based programmers
./flash.sh arduinoisp /dev/ttyUSB0  # Arduino as ISP
./flash.sh qyf0893             # QYF0893/Digispark boards
```

## Prerequisites

### Required Tools

1. **arduino-cli** - Arduino command-line interface

   - Install: `curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh`
2. **avrdude** - AVR programmer utility

   - Install: `sudo apt-get install avrdude`
3. **ATtiny Core** - Board support package (installed automatically by script)

   - Core: `attiny:avr`

### Verify Installation

```bash
arduino-cli version
avrdude -v
```

## Supported Programmers

The flash script supports multiple programming methods:

| Programmer     | Type        | Port Required          | Notes                                  |
| -------------- | ----------- | ---------------------- | -------------------------------------- |
| `arduinoisp` | External    | Yes (`/dev/ttyUSB0`) | Arduino Uno/Nano as ISP                |
| `usbasp`     | USB         | No                     | USBasp programmer, QYF0893             |
| `usbtiny`    | USB         | No                     | USBtinyISP programmer                  |
| `avrisp2`    | External    | Yes                    | Atmel AVR ISP mkII                     |
| `qyf0893`    | USB         | No                     | QYF0893 USB board (alias for usbasp)   |
| `digispark`  | USB         | No                     | Digispark USB board (alias for usbasp) |
| `auto`       | Auto-detect | No                     | Automatically detects USB programmers  |

### Bootloader Installation

Before using USB to TTL serial programming, you must install a bootloader on the ATtiny85:

```bash
./burn_bootloader.sh [port]
```

This script:

1. Downloads/installs Optiboot bootloader for ATtiny85
2. Sets appropriate fuses for serial programming
3. Burns the bootloader using Arduino ISP
4. Enables 9600 baud serial programming

**Requirements:** Arduino configured as ISP programmer.

#### Hardware Setup for Bootloader Burning

**Required Components:**
- Arduino Uno/Nano (programmer)
- ATtiny85 microcontroller chip
- Breadboard or development board
- 6-pin Dupont cable or jumper wires
- Power supply for ATtiny85 (optional, Arduino can provide power)

**Step 1: Prepare Arduino as ISP Programmer**

1. Open Arduino IDE
2. Go to: `File > Examples > ArduinoISP > ArduinoISP`
3. Select your Arduino board (Uno/Nano) and correct port (`/dev/ttyACM0`)
4. Upload the ArduinoISP sketch to your Arduino
5. Arduino is now configured as ISP programmer

**Step 2: Wire ATtiny85 to Arduino ICSP Header**

Use the 6-pin ICSP header on Arduino Uno/Nano:

```
Arduino ICSP Header     ATtiny85 Pins
┌─────────────────┐     ┌─────────┐
│ 1 ▫ MISO  ▫ 2 │     │ ▫ ▫ ▫ ▫ │
│ 3 ▫ SCK   ▫ 4 │     │ ▫ ▫ ▫ ▫ │
│ 5 ▫ RESET ▫ 6 │     │ ▫ ▫ ▫ ▫ │
└─────────────────┘     └─────────┘
     │ │ │ │ │ │         │ │ │ │ │ │
     │ │ │ │ │ │         │ │ │ │ │ │
    VCC GND RST MOSI MISO SCK
          │   │   │   │   │
          │   │   │   │   │
         5V  GND RST PB0 PB1 PB2
              (ATtiny85 pins: 8,4,1,5,6,7)
```

**ICSP Pin Mapping:**
- ICSP 1 (MISO) → ATtiny85 Pin 6 (PB1)
- ICSP 2 (VCC) → ATtiny85 Pin 8 (VCC)
- ICSP 3 (SCK) → ATtiny85 Pin 7 (PB2)
- ICSP 4 (MOSI) → ATtiny85 Pin 5 (PB0)
- ICSP 5 (RST) → ATtiny85 Pin 1 (RST)
- ICSP 6 (GND) → ATtiny85 Pin 4 (GND)

**Step 3: Power the ATtiny85**

The Arduino ICSP header provides programming signals but may not power the ATtiny85 sufficiently. Add:
- Connect ATtiny85 Pin 8 (VCC) to 5V power source
- Connect ATtiny85 Pin 4 (GND) to ground

**Step 4: Run Bootloader Script**

```bash
./burn_bootloader.sh
```

The script will auto-detect your Arduino and attempt to burn the bootloader.

## Detailed Instructions

### Method 1: Arduino as ISP (Most Common for External Programmer)

**Hardware Setup:**

- Upload "ArduinoISP" sketch to Arduino Uno/Nano
- Wire Arduino to ATtiny85 (see [Hardware Setup](#hardware-setup))

**Flashing:**

```bash
./flash.sh arduinoisp /dev/ttyUSB0
```

Replace `/dev/ttyUSB0` with your actual serial port.

**Find Serial Port:**

```bash
ls -la /dev/ttyUSB* /dev/ttyACM*
# or
arduino-cli board list
```

### Method 2: USBasp Programmer

**Hardware Setup:**

- Connect USBasp programmer to ATtiny85 via ISP header
- Connect USBasp to computer via USB

**Flashing:**

```bash
./flash.sh usbasp
```

No port specification needed for USB programmers.

### Method 3: QYF0893 USB Development Board

The QYF0893 is an ATtiny85-based USB development board that can be programmed directly via USB.

**Hardware Setup:**

- Connect QYF0893 to computer via USB cable
- No external programmer needed

**Flashing:**

```bash
# Auto-detect (recommended)
./flash.sh auto

# Or explicitly specify
./flash.sh qyf0893
# or
./flash.sh usbasp
```

**Verifying Connection:**

```bash
lsusb
# Look for USBasp or USBtinyISP devices
```

### Method 4: Auto-Detection

The script can automatically detect USB-based programmers:

```bash
./flash.sh auto
```

This will:

1. Scan for USB programmers (USBasp, USBtinyISP)
2. Automatically use the detected programmer
3. Fall back to ArduinoISP if no USB programmer found

### Method 5: USB to TTL Serial Programming

**Important:** USB to TTL programming requires a serial bootloader to be pre-installed on the ATtiny85. Standard factory ATtiny85 chips cannot be programmed directly with USB to TTL - they must first be programmed via ISP (ArduinoISP, USBasp, etc.) to install a serial bootloader.

**Requirements:**

- ATtiny85 with serial bootloader (e.g., Micronucleus, Optiboot, or custom serial bootloader)
- USB to TTL adapter (CP2102, CH340, FT232RL, etc.)
- Serial communication software (avrdude, Arduino IDE, etc.)

**Hardware Setup:**
Connect USB to TTL adapter to ATtiny85:

| USB to TTL Pin | ATtiny85 Pin | Function | Notes                                    |
| -------------- | ------------ | -------- | ---------------------------------------- |
| VCC (3.3V/5V)  | Pin 8 (VCC)  | Power    | Use 5V for ATtiny85                      |
| GND            | Pin 4 (GND)  | Ground   | Common ground                            |
| TX             | Pin 6 (PB1)  | RX       | Serial receive (software UART)           |
| RX             | Pin 5 (PB0)  | TX       | Serial transmit (software UART)          |
| RTS/DTR        | Pin 1 (RST)  | Reset    | Optional, for auto-reset (via capacitor) |

**Note:** Some USB to TTL adapters have RTS/DTR pins that can be used for auto-reset. Connect through a 0.1μF capacitor to the reset pin if available.

**Flashing:**

```bash
# Using avrdude with serial protocol
avrdude -p attiny85 -c arduino -P /dev/ttyUSB0 -b 9600 -U flash:w:compiled/attiny85.ino.hex:i

# Adjust port and baud rate according to your bootloader
```

**Bootloader Installation:**
To enable USB to TTL programming, first install a serial bootloader using ISP:

1. Use ArduinoISP or USBasp to program the bootloader
2. Common bootloaders for ATtiny85: Micronucleus, Optiboot (modified), or custom implementations
3. After bootloader installation, USB to TTL can be used for subsequent programming

**Limitations:**

- Requires initial ISP programming to install bootloader
- Slower than ISP programming
- Limited to bootloaders that support serial upload
- May require manual reset timing

## QYF0893 USB Development Board

### Overview

The QYF0893 is a USB development board similar to Digispark, featuring:

- ATtiny85 microcontroller
- Built-in USB interface
- Direct USB programming (no external programmer required)
- Minimal external components

### Programming Methods

#### Method 1: USBasp Protocol (Default)

Most QYF0893 boards use USBasp-compatible protocol:

```bash
./flash.sh qyf0893
# or
./flash.sh usbasp
```

#### Method 2: Micronucleus Bootloader

If your board has Micronucleus bootloader installed:

**Install Micronucleus:**

```bash
sudo apt-get install micronucleus
```

**Flash:**
The script will automatically try Micronucleus if USBasp fails, or manually:

```bash
micronucleus --run compiled/attiny85.ino.hex
```

**Note:** With Micronucleus, you may need to plug in the board when prompted (5-second window).

#### Method 3: USBtinyISP Protocol

Some boards use USBtinyISP protocol:

```bash
./flash.sh usbtiny
```

### QYF0893 Pin Configuration

After flashing, the ATtiny85 uses:

- **Pin 2 (PB3)**: RPM Hall Sensor input (interrupt)
- **Pin 5 (PB0)**: PWM output to ESC
- **Pin 8 (VCC)**: 5V USB power (via USB connector)
- **Pin 4 (GND)**: Ground (via USB connector)

The USB connector provides both programming and power.

### Troubleshooting QYF0893

**Device Not Detected:**

1. Use a data USB cable (not charge-only)
2. Try different USB port
3. Power cycle the board (unplug/reconnect)
4. Check `lsusb` for device detection

**Permission Issues:**

```bash
sudo usermod -a -G dialout $USER
# Log out and back in, or:
newgrp dialout
```

**Programming Mode:**
Some boards require button press to enter programming mode. Check your board's documentation.

## Hardware Setup

### Arduino as ISP Wiring

Connect Arduino (programmer) to ATtiny85:

| Arduino Pin | ATtiny85 Pin | Function |
| ----------- | ------------ | -------- |
| Pin 10      | Pin 1 (RST)  | Reset    |
| Pin 11      | Pin 5 (PB0)  | MOSI     |
| Pin 12      | Pin 6 (PB1)  | MISO     |
| Pin 13      | Pin 7 (PB2)  | SCK      |
| 5V          | Pin 8 (VCC)  | Power    |
| GND         | Pin 4 (GND)  | Ground   |

#### Using Arduino ICSP Header (Recommended)

For easier connection, use the 6-pin ICSP header on Arduino Uno/Nano:

| ICSP Pin | Arduino Pin | ATtiny85 Pin | Function | Color Code |
| -------- | ----------- | ------------ | -------- | ---------- |
| 1        | Pin 12      | Pin 6 (PB1)  | MISO     | Brown      |
| 2        | 5V          | Pin 8 (VCC)  | Power    | Red        |
| 3        | Pin 13      | Pin 7 (PB2)  | SCK      | Orange     |
| 4        | Pin 11      | Pin 5 (PB0)  | MOSI     | Yellow     |
| 5        | Pin 10      | Pin 1 (RST)  | Reset    | Green      |
| 6        | GND         | Pin 4 (GND)  | Ground   | Black      |

**Connection Method:**

- Use a 6-pin Dupont cable or jumper wires
- Connect directly from Arduino ICSP header to ATtiny85
- Ensure proper orientation (pin 1 marked on most ICSP headers)

**Cable Color Coding (optional but helpful):**

- Brown: MISO
- Red: VCC (+5V)
- Orange: SCK
- Yellow: MOSI
- Green: RESET
- Black: GND

**Important:** Upload "ArduinoISP" sketch to Arduino first:

1. Open Arduino IDE
2. File > Examples > ArduinoISP > ArduinoISP
3. Select board and port
4. Upload sketch

### USBasp Wiring

Connect USBasp to ATtiny85 via 6-pin ISP header:

| USBasp Pin | ATtiny85 Pin | Function |
| ---------- | ------------ | -------- |
| MOSI       | Pin 5 (PB0)  | MOSI     |
| MISO       | Pin 6 (PB1)  | MISO     |
| SCK        | Pin 7 (PB2)  | SCK      |
| RST        | Pin 1 (RST)  | Reset    |
| VCC        | Pin 8 (VCC)  | Power    |
| GND        | Pin 4 (GND)  | Ground   |

**Note:** Ensure correct pin orientation (usually marked with arrow or pin 1 indicator).

### Production Wiring (After Flashing)

After programming, wire for operation:

| ATtiny85 Pin | Connection  | Function                         |
| ------------ | ----------- | -------------------------------- |
| Pin 2 (PB3)  | Hall Sensor | RPM feedback (interrupt input)   |
| Pin 5 (PB0)  | ESC Signal  | PWM output to motor controller   |
| Pin 8 (VCC)  | 5V Supply   | Power (2.7-5.5V, recommended 5V) |
| Pin 4 (GND)  | Ground      | Common ground                    |

## Troubleshooting

### Common Issues

#### Compilation Errors

**Error: "board attiny:avr:attiny85 not found"**

```bash
# Install ATtiny core
arduino-cli core update-index
arduino-cli core install attiny:avr
```

**Error: Missing header files**

- Ensure all `.h` files are in the sketch directory:
  - `config.h`
  - `config_common.h`
  - `pid_common.h`
  - `rpm_common.h`
  - `isr_common.h`

#### Flashing Errors

**Error: "Permission denied"**

```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

**Error: "Cannot find USB device"**

- Verify USB cable supports data (not charge-only)
- Try different USB port
- Check `lsusb` for device detection
- Ensure device is powered

**Error: "Device not responding"**

- Check wiring connections
- Verify programmer is working
- Try different programmer type
- Check fuse settings haven't locked the device

**Error: "Wrong microcontroller"**

- Verify you're using ATtiny85 (not ATtiny45 or ATtiny25)
- Check device signature with: `avrdude -p attiny85 -c [programmer] -v`

#### Port Issues

**Finding Serial Port:**

```bash
ls -la /dev/ttyUSB* /dev/ttyACM*
arduino-cli board list
dmesg | grep -i tty | tail -10
```

**Port Permission:**

```bash
sudo chmod 666 /dev/ttyUSB0  # Temporary fix
# Or add user to dialout group (permanent)
```

### Advanced Troubleshooting

#### Verify Device Signature

```bash
avrdude -p attiny85 -c [programmer] -v
```

Should show:

```
Device signature = 0x1e930b
```

#### Read Fuses

```bash
avrdude -p attiny85 -c [programmer] -U lfuse:r:-:h -U hfuse:r:-:h -U efuse:r:-:h
```

Expected for 8MHz internal oscillator:

- lfuse: 0xE2
- hfuse: 0xDF
- efuse: 0xFF

#### Verify Flash

Read back flash to verify programming:

```bash
avrdude -p attiny85 -c [programmer] -U flash:r:verify.hex:i
# Compare with original hex file
diff compiled/attiny85.ino.hex verify.hex
```

## Manual Flashing

If the automated script fails, you can flash manually:

### Step 1: Compile

```bash
cd /home/azzar/project/PIDs/PID_simple_controll/attiny85
arduino-cli compile --fqbn attiny:avr:ATtinyX5 --build-path compiled .
```

### Step 2: Set Fuses (8MHz Internal Oscillator)

**ArduinoISP:**

```bash
avrdude -p attiny85 -c arduinoisp -P /dev/ttyUSB0 -b 19200 \
  -U lfuse:w:0xE2:m -U hfuse:w:0xDF:m -U efuse:w:0xFF:m
```

**USBasp:**

```bash
avrdude -p attiny85 -c usbasp \
  -U lfuse:w:0xE2:m -U hfuse:w:0xDF:m -U efuse:w:0xFF:m
```

**USBtinyISP:**

```bash
avrdude -p attiny85 -c usbtiny \
  -U lfuse:w:0xE2:m -U hfuse:w:0xDF:m -U efuse:w:0xFF:m
```

### Step 3: Flash Program

**ArduinoISP:**

```bash
avrdude -p attiny85 -c arduinoisp -P /dev/ttyUSB0 -b 19200 \
  -U flash:w:compiled/attiny85.ino.hex:i
```

**USBasp:**

```bash
avrdude -p attiny85 -c usbasp \
  -U flash:w:compiled/attiny85.ino.hex:i
```

**USBtinyISP:**

```bash
avrdude -p attiny85 -c usbtiny \
  -U flash:w:compiled/attiny85.ino.hex:i
```

**Micronucleus (if bootloader installed):**

```bash
micronucleus --run compiled/attiny85.ino.hex
```

### Step 4: Verify

```bash
avrdude -p attiny85 -c [programmer] -U flash:v:compiled/attiny85.ino.hex:i
```

## Fuse Settings

The script automatically sets fuses for 8MHz internal oscillator operation:

- **lfuse: 0xE2** - Internal 8MHz oscillator, no clock divide, no startup delay
- **hfuse: 0xDF** - SPI programming enabled, no watchdog reset, no brown-out detector
- **efuse: 0xFF** - Self-programming disabled

**Warning:** Incorrect fuse settings can lock your device. The script uses safe defaults.

## Program Size

The compiled program uses:

- **Flash:** 1852 bytes (90% of 2048 bytes)
- **SRAM:** 55 bytes (42% of 128 bytes)

This leaves minimal room for expansion but fits comfortably within ATtiny85 limits.

## Additional Resources

- **Arduino CLI Documentation:** https://arduino.github.io/arduino-cli/
- **AVRDUDE Manual:** `man avrdude`
- **ATtiny85 Datasheet:** Search for "ATtiny85 datasheet"
- **Project README:** See `README.md` for hardware and software details

## Support

For issues specific to this project:

1. Check this guide's troubleshooting section
2. Review compilation/flashing errors carefully
3. Verify hardware connections
4. Check programmer compatibility

For general ATtiny85 programming:

- ATtiny Core repository
- Arduino forums
- AVR Freaks community
