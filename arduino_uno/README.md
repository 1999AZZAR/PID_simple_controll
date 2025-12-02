# Arduino Uno BLDC PID Controller

**Simplified PID controller** for BLDC motors with potentiometer tuning and serial monitoring.

## Table of Contents

- [Overview](#overview)
- [Quick Start](#quick-start)
- [Project Structure](#project-structure)
- [Hardware Setup](#hardware-setup)
- [Configuration](#configuration)
- [Operating Modes](#operating-modes)
- [Serial Plotter Output](#serial-plotter-output)
- [Tuning Procedure](#tuning-procedure)
- [Troubleshooting](#troubleshooting)
- [Performance Notes](#performance-notes)

## Overview

The Arduino Uno version provides a streamlined PID controller for BLDC motors with potentiometer-based tuning and serial monitoring output.

### Key Features

- **Fixed Target Speed**: 1440 RPM constant speed control
- **Potentiometer Tuning**: 4 potentiometers for real-time parameter adjustment
- **Serial Plotter Integration**: Real-time visualization of control performance
- **Modular Configuration**: All settings centralized in `config.h`
- **Two Operating Modes**: Production (fixed parameters) and Potentiometer Tuning
- **Simplified Design**: No serial commands or EEPROM storage for maximum reliability

## Quick Start

1. **Hardware Setup**: Connect components according to pin assignments below
2. **Upload Code**: Load `arduino_uno.ino` to Arduino Uno
3. **Configure**: Modify `config.h` for your specific setup
4. **Tune Parameters**: Use potentiometers for real-time adjustment
5. **Monitor**: Use Serial Plotter for real-time feedback

## Project Structure

```
arduino_uno/
├── arduino_uno.ino          # Main Arduino sketch
├── config.h                 # Configuration header (all settings)
├── hardware_schematic.md   # Hardware setup guide
└── README.md               # This documentation
```

## Hardware Setup

### Required Components
- Arduino Uno (or compatible board)
- 3-Hall BLDC motor (42BLF20-22.0223 or equivalent)
- BLDC motor controller (ESC) compatible with motor
- SPDT switch or jumper (mode selection)

### Optional Components (for potentiometer tuning)
- 4x 10kΩ potentiometers
- Breadboard and jumper wires

### Pin Connections

| Component | Arduino Pin | Description |
|-----------|-------------|-------------|
| BLDC Hall Sensor | Digital Pin 2 | Any Hall wire from motor (interrupt pin) - provides 2 pulses per electrical revolution |
| PWM Output | Digital Pin 9 | PWM signal to ESC |
| Mode Switch | Digital Pin 3 | LOW = Potentiometer mode, HIGH = Production/Serial mode |
| PPR Pot | Analog A0 | Pulses per revolution (1-100) |
| Kp Pot | Analog A1 | Proportional gain (0-2.0) |
| Ki Pot | Analog A2 | Integral gain (0-1.0) |
| Kd Pot | Analog A3 | Derivative gain (0-0.1) |
| A4 | Analog A4 | Available for future use (I2C, etc.) |

### Hall Sensor Signal Options

The controller uses a single Hall sensor wire for RPM feedback, but for enhanced performance, you can implement a composite signal from all three Hall sensors:

#### Single Hall Sensor (Current Implementation)
- Connect any one Hall wire (A, B, or C) to Arduino Pin 2
- Provides 2 pulses per electrical revolution
- Simple wiring, reliable operation

#### Composite Hall Sensor Signal (Advanced)
- Combine all three Hall sensors using OR logic
- Provides 6 pulses per electrical revolution
- Higher resolution and smoother motor control
- Requires external OR gate or additional microcontroller pins

**Implementation Example for Composite Signal:**
```cpp
// Hardware OR gate connection
// Hall A → Diode → Common line → Arduino Pin 2
// Hall B → Diode → Common line
// Hall C → Diode → Common line
// Pulldown resistor on common line
```

## Configuration

All Arduino Uno settings are centralized in `config.h`:

### Pin Definitions
```cpp
#define RPM_SENSOR_PIN      2   // Hall sensor input (interrupt)
#define PWM_OUTPUT_PIN      9   // PWM output to ESC
#define MODE_SWITCH_PIN     3   // Mode selection switch
#define POT_PULSES_PER_REV  A0  // Pulses per revolution potentiometer
#define POT_KP              A1  // Proportional gain pot
#define POT_KI              A2  // Integral gain pot
#define POT_KD              A3  // Derivative gain pot
// A4 available for future use (I2C, etc.)
```

### Control Parameters
```cpp
#define PRODUCTION_TARGET_RPM 1440.0  // Default target RPM
#define PRODUCTION_KP         0.5     // Default proportional gain
#define PRODUCTION_KI         0.1     // Default integral gain
#define PRODUCTION_KD         0.01    // Default derivative gain
#define PULSES_PER_REV          6     // Hall sensor pulses per revolution
```

### Debug and Safety Settings
```cpp
#define DEBUG_MODE_ENABLED     false  // Enable detailed logging
#define SOFT_START_DURATION_MS  2000  // Soft-start ramp time
#define MIN_PULSE_WIDTH_US      100   // Debounce filter threshold

// Safety features (recommended to keep enabled)
#define WATCHDOG_ENABLED           true   // Hardware watchdog protection (4s timeout)
// Emergency stop feature removed for simplified operation
```

## Operating Modes

The Arduino Uno version supports two operating modes selected by the mode switch (Digital Pin 3):

### Production Mode (Default)
- **Mode Switch**: HIGH or floating
- **Behavior**: Uses fixed target RPM (1440 RPM) with potentiometer-adjustable PID parameters
- **Features**: Stable, predictable operation
- **Use Case**: Final production deployment

### Potentiometer Tuning Mode
- **Mode Switch**: LOW (connected to GND)
- **Behavior**: Real-time parameter adjustment via 4 potentiometers
- **Monitoring**: Serial Plotter shows live control response
- **Use Case**: Initial PID tuning and testing

## Serial Plotter Output

The system outputs seven comma-separated values for Serial Plotter visualization:
```
Target,Current,Error,PID_Output,Kp,Ki,Kd,PPR
```

- **Target**: Desired RPM setpoint
- **Current**: Measured motor RPM
- **Error**: Difference between target and current
- **PID_Output**: Computed PID control value
- **Kp**: Proportional gain
- **Ki**: Integral gain
- **Kd**: Derivative gain
- **PPR**: Pulses per revolution

### Plotter Setup
1. Open Arduino IDE → Tools → Serial Plotter
2. Set baud rate to 115200
3. Seven traces will display real-time control performance and parameters

## Tuning Procedure

### Potentiometer Tuning
1. Set mode switch to LOW (potentiometer mode)
2. Open Serial Plotter (115200 baud)
3. Adjust potentiometers while monitoring response:
   - **Pot A0 (PPR)**: Pulses per revolution (1-100 range)
   - **Pot A1 (Kp)**: Proportional gain (0-2.0 range)
   - **Pot A2 (Ki)**: Integral gain (0-1.0 range)
   - **Pot A3 (Kd)**: Derivative gain (0-0.1 range)
4. Record optimal potentiometer positions
5. Transfer values to production constants in `config.h`

## Troubleshooting

### Motor Not Starting
- Verify PWM output pin (Digital Pin 9) connected to ESC signal input
- Check ESC power supply and motor connections
- Confirm Hall sensor providing pulses (monitor with Serial Plotter)
- Verify PID parameters not set to extreme values

### No RPM Reading
- Confirm Hall sensor wire connected to Digital Pin 2
- Check sensor power (5V) and ground connections
- Verify `PULSES_PER_REV` matches your motor specifications (fixed at compile-time)
- Test with oscilloscope on sensor pin for pulse signals

### Unstable Control
- Reduce Kp gain first, then adjust Ki and Kd
- Check for electrical noise on sensor lines
- Verify proper motor/ESC grounding
- Ensure clean power supplies for Arduino and motor

## Performance Notes

- **Memory Usage**: ~18% RAM usage (optimized for reliability)
- **Control Frequency**: 100Hz main loop, 10Hz RPM calculation
- **PWM Frequency**: ~490Hz (Timer0 with prescaler 8)
- **Interrupt Response**: Hall sensor debounce filter prevents false triggers
- **Simplified Design**: No EEPROM wear or serial command overhead
