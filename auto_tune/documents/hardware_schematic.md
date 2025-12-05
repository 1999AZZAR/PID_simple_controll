# BLDC PID Controller - Python GUI Hardware Schematic

## Table of Contents

- [System Overview](#system-overview)
- [Required Hardware](#required-hardware)
- [Arduino Uno Pin Configuration](#arduino-uno-pin-configuration)
- [Serial Communication Setup](#serial-communication-setup)
- [BLDC Motor Connections](#bldc-motor-connections)
- [Power Supply Configuration](#power-supply-configuration)
- [Wiring Diagrams](#wiring-diagrams)
- [Hardware Testing](#hardware-testing)
- [Troubleshooting](#troubleshooting)

## System Overview

**Python GUI Control System**: This hardware schematic describes the Arduino Uno setup for the Python GUI version of the BLDC PID controller. Unlike the standalone Arduino version with potentiometers, this system uses serial communication for real-time parameter adjustment and monitoring.

### Key Differences from Standalone Version
- **No Potentiometers**: All PID tuning via Python GUI
- **Serial Communication**: Bidirectional data exchange with computer
- **Real-time Monitoring**: Live plotting and parameter feedback
- **Configuration Persistence**: Parameters saved to Arduino EEPROM
- **Advanced Testing**: PPR verification and diagnostic capabilities

## Required Hardware

### Core Components
- **Arduino Uno** (or compatible board with ATmega328P)
- **BLDC Motor** with 3-Hall sensors (e.g., 42BLF20-22.0223)
- **BLDC Motor Controller (ESC)** compatible with motor voltage/current
- **USB Cable** (Type A to Type B) for Arduino-computer connection
- **Power Supply** for Arduino (7-12V recommended)

### Optional Components
- **Logic Level Shifter** (if motor Hall sensors operate at different voltage)
- **Pull-up Resistors** (4.7kΩ) for Hall sensor lines
- **Decoupling Capacitors** (10µF, 0.1µF) for power supply filtering
- **Shielded Cable** for Hall sensor connections (if EMI issues occur)

## Arduino Uno Pin Configuration

### Primary Connections (Required)
```
Arduino Uno Pin Mapping - Python GUI Version:
├── Digital Pin 2: RPM Sensor Input (Interrupt 0) ← Any Hall wire from BLDC motor
├── Digital Pin 9: PWM Output → ESC signal input
├── USB Port: Serial Communication ↔ Computer running Python GUI
├── 5V: Power for Hall sensors
└── GND: Common ground with motor/ESC
```

### Detailed Pin Functions

#### Digital Pin 2 (RPM Input)
- **Function**: Hall sensor period measurement
- **Interrupt**: Hardware interrupt 0 (rising edge triggered)
- **Debounce**: 100µs minimum pulse width filtering
- **Pull-up**: Internal INPUT_PULLUP enabled
- **Connection**: Any single Hall wire (A, B, or C) from BLDC motor

#### Digital Pin 9 (PWM Output)
- **Function**: Motor speed control signal to ESC
- **Timer**: Timer1 with PWM frequency ~490Hz
- **Resolution**: 8-bit (0-255 duty cycle)
- **Protection**: PWM constrained to safe operating range
- **Connection**: ESC signal input pin

#### Serial Communication (USB)
- **Baud Rate**: 115200 bps (fixed)
- **Protocol**: Custom command/response format
- **Buffer**: 64-byte hardware buffer
- **Flow Control**: None (sufficient for low data rates)
- **Connection**: USB cable to computer running Python GUI

## Serial Communication Setup

### Physical Connection
1. **USB Cable**: Connect Arduino Uno to computer USB port
2. **Driver Installation**: Automatic on Windows/macOS, manual on Linux
3. **Port Detection**: GUI automatically scans available serial ports
4. **Permission Setup**: Linux users add to `dialout` group

### Communication Parameters
```
Serial Configuration:
├── Baud Rate: 115200
├── Data Bits: 8
├── Stop Bits: 1
├── Parity: None
├── Flow Control: None
└── Timeout: 1 second
```

### Command Protocol
**Python GUI → Arduino Commands:**
```
SET_KP <value>              # Set proportional gain
SET_KI <value>              # Set integral gain
SET_KD <value>              # Set derivative gain
SET_TARGET_RPM <value>      # Set target RPM
SET_PULSES_PER_REV <value>  # Set pulses per revolution (fixed at 4 for 8-pole motors)
ENABLE_MOTOR <0|1>          # Enable/disable motor
RESET_CONTROLLER            # Reset PID state
GET_STATUS                  # Request status update
```

**Arduino → Python GUI Status Response:**
```
STATUS:timestamp,target_rpm,current_rpm,error,pid_output,kp,ki,kd,ppr,motor_enabled
```

## BLDC Motor Connections

### Hall Sensor Wiring
**3-Hall BLDC Motor Compatibility:**
```
BLDC Motor Hall Sensors:
├── Hall A (typically Yellow) → Arduino Pin 2 (OR)
├── Hall B (typically Green)  → Arduino Pin 2 (OR)
├── Hall C (typically Blue)   → Arduino Pin 2 (OR)
├── VCC (typically Red)      → Arduino 5V
└── GND (typically Black)    → Arduino GND

Note: Connect ANY ONE Hall wire to Arduino Pin 2
```

### ESC Connection
**Electronic Speed Controller:**
```
ESC Connections:
├── Signal Input → Arduino Pin 9 (PWM)
├── Power Input → Motor power supply
├── Motor Output → BLDC motor phases
└── Ground → Common ground
```

### Motor Power Supply
**Separate Power Systems:**
```
Power Supply Configuration:
├── Arduino Power: 7-12V via barrel jack or VIN pin
├── Motor Power: Appropriate voltage for motor/ESC
├── Common Ground: Connect Arduino GND to motor GND
└── Isolation: Keep logic and motor power separate
```

## Power Supply Configuration

### Recommended Setup
```
Dual Power Supply System:
├── Logic Power (Arduino):
│   ├── Voltage: 7-12V DC
│   ├── Current: 500mA minimum
│   ├── Source: USB or external adapter
│   └── Protection: Arduino onboard regulation
│
└── Motor Power (ESC/Motor):
    ├── Voltage: Motor-specific (check datasheet)
    ├── Current: 2x motor rated current minimum
    ├── Source: Separate DC power supply
    └── Protection: ESC onboard features
```

### Voltage Compatibility
- **Hall Sensors**: Operate at 5V (Arduino compatible)
- **ESC Signal**: 5V PWM signal (Arduino compatible)
- **Motor Phases**: High voltage/current (ESC handles conversion)
- **Common Ground**: Essential for proper operation

## Wiring Diagrams

### Basic Connection Diagram
```
Computer (USB) ↔ Arduino Uno ↔ BLDC Motor System
     ↓              ↓              ↓
Python GUI    Serial Comm     Hall Sensors + ESC
     ↓              ↓              ↓
Real-time      STATUS: data    Motor Control
Monitoring     Commands        PWM Output
```

### Detailed Wiring Schematic
```
Arduino Uno Connections:

USB Port (Serial Communication)
├── TX → Computer RX
├── RX → Computer TX
├── DTR → Arduino Reset (programming)
└── USB 5V → Arduino Power (optional)

Digital Pin 2 (RPM Sensor)
├── INPUT_PULLUP enabled
├── Interrupt 0 (rising edge)
├── 100µs debounce filtering
└── Hall sensor wire connection

Digital Pin 9 (PWM Output)
├── Timer1 PWM output
├── ~490Hz frequency
├── 0-255 duty cycle range
└── ESC signal input

Power Connections
├── VIN/Barrel Jack: 7-12V input
├── 5V: Hall sensor power
├── 3.3V: Not used
├── GND: Common ground
└── RESET: Programming/reset pin
```

### Connection Verification Checklist
- [ ] Arduino USB cable securely connected
- [ ] Hall sensor wire connected to Pin 2
- [ ] ESC signal wire connected to Pin 9
- [ ] Hall sensor power (5V) connected
- [ ] Common ground established
- [ ] Motor power supply connected to ESC
- [ ] BLDC motor connected to ESC

## Hardware Testing

### Serial Communication Test
1. **Upload Arduino Code**: Load `auto_tune/code/code.ino`
2. **Open Serial Monitor**: Arduino IDE → Tools → Serial Monitor
3. **Set Baud Rate**: 115200
4. **Verify Output**: Should see "BLDC PID Controller Started"
5. **Send Command**: Type `GET_STATUS` and press Enter
6. **Check Response**: Should receive STATUS: message

### PWM Output Test
1. **Disconnect Motor**: Remove motor from ESC
2. **Monitor PWM Pin**: Use oscilloscope or LED test
3. **Send PWM Command**: `ENABLE_MOTOR 1` via Serial Monitor
4. **Verify Signal**: PWM signal on Pin 9
5. **Test Range**: Signal should vary with PID output

### RPM Sensor Test
1. **Manual Rotation**: Spin motor shaft by hand
2. **Monitor Serial**: Watch for RPM changes in STATUS messages
3. **Verify PPR**: PPR is fixed at 4 for 8-pole BLDC with single Hall sensor

### Full System Test
1. **Power On**: Arduino and motor power supplies
2. **Launch GUI**: `python control.py`
3. **Connect Serial**: Select port and click "Connect"
4. **Enable Motor**: Click "Enable Motor" button
5. **Monitor Plots**: Verify real-time data updates
6. **Test Controls**: Adjust PID parameters and observe response

## Troubleshooting

### Serial Connection Issues

#### Port Not Detected
**Symptoms:** Python GUI shows no available ports
```
Windows:
├── Install Arduino drivers
├── Try different USB ports
├── Run Arduino IDE first
└── Check Device Manager

macOS:
├── Grant USB permissions
├── Check /dev/tty.* devices
├── Reset Arduino (press reset button)
└── Try different USB cables

Linux:
├── Add user to dialout group: sudo usermod -a -G dialout $USER
├── Logout and login again
├── Check /dev/ttyACM* permissions
└── sudo chmod 666 /dev/ttyACM0
```

#### No Data Received
**Symptoms:** Connected but no STATUS messages
```
Possible Causes:
├── Wrong baud rate (must be 115200)
├── Arduino code not uploaded
├── Serial buffer overflow
├── USB cable issues
└── Arduino reset during connection
```

### Motor Control Issues

#### Motor Not Starting
**Symptoms:** Motor doesn't respond to Enable command
```
Debug Steps:
├── Check PWM signal on Pin 9
├── Verify ESC power and connections
├── Test ESC with direct PWM signal
├── Check Hall sensor connections
└── Verify motor/ESC compatibility
```

#### Unstable Motor Speed
**Symptoms:** Speed varies, oscillations, or hunting
```
PID Tuning Issues:
├── Reduce Kp gain (start lower)
├── Check PID parameter ranges
├── Verify PPR setting accuracy
├── Check for electrical interference
└── Test with known stable parameters
```

#### Incorrect RPM Readings
**Symptoms:** Displayed RPM doesn't match actual speed
```
Calibration Issues:
├── Verify PPR setting (6 for 3-Hall BLDC)
├── Check Hall sensor connections
├── Test with PPR verification command
├── Verify Hall sensor power (5V)
└── Check for EMI interference
```

### Hardware Reliability Issues

#### EMI Interference
**Symptoms:** Erratic RPM readings or motor behavior
```
Solutions:
├── Use shielded cable for Hall sensors
├── Separate Hall wires from PWM wires
├── Add ferrite beads on signal lines
├── Improve grounding connections
└── Add decoupling capacitors
```

#### Power Supply Noise
**Symptoms:** System instability or reset issues
```
Solutions:
├── Use separate supplies for logic and motor
├── Add large electrolytic capacitors (100µF+)
├── Add small ceramic capacitors (0.1µF)
├── Improve cable routing
└── Use regulated power supplies
```

### Performance Optimization

#### Communication Latency
- **Serial Delay**: ~10-20ms round-trip delay is normal
- **GUI Responsiveness**: Local changes are immediate
- **Plot Updates**: 4-5Hz refresh rate optimized for performance
- **Buffer Size**: 25 data points balances responsiveness and memory

#### System Resources
- **Arduino Memory**: ~70% flash, ~25% RAM usage
- **Python Memory**: 50-100MB depending on session length
- **CPU Usage**: 5-15% on host computer
- **Serial Bandwidth**: <1KB/s for STATUS updates

---

## Technical Specifications

### Arduino Uno Compatibility
- **Microcontroller**: ATmega328P
- **Clock Speed**: 16MHz
- **Flash Memory**: 32KB (22KB available)
- **SRAM**: 2KB (1.5KB available)
- **EEPROM**: 1KB (512B available)

### Serial Communication Limits
- **Baud Rate**: 115200 bps maximum
- **Buffer Size**: 64 bytes hardware buffer
- **Command Length**: <32 characters recommended
- **Response Time**: <10ms for most commands

### PWM Output Specifications
- **Frequency**: ~490Hz (Timer1 prescaler 8)
- **Resolution**: 8-bit (256 steps)
- **Voltage**: 5V logic level
- **Current**: <40mA (Arduino limitation)

### RPM Sensing Capabilities
- **Interrupt Response**: <5µs
- **Debounce Filter**: 100µs minimum
- **Maximum RPM**: Limited by Hall sensor frequency
- **Accuracy**: ±1% with proper PPR calibration

---

**This hardware schematic provides complete setup instructions for the Python GUI version of the BLDC PID controller, ensuring reliable operation and accurate motor speed control.**

**Date**: December 2025
**Compatibility**: Arduino Uno R3 and compatible boards
**Software Version**: Python GUI v1.0
