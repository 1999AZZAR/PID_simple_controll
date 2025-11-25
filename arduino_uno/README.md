# Arduino Uno BLDC PID Controller

**Development implementation** for the BLDC Motor PID Controller - Full-featured environment with real-time tuning capabilities.

## Table of Contents

- [Overview](#overview)
- [Quick Start](#quick-start)
- [Project Structure](#project-structure)
- [Hardware Setup](#hardware-setup)
- [Configuration](#configuration)
- [Operating Modes](#operating-modes)
- [Serial Commands](#serial-commands)
- [Serial Plotter Output](#serial-plotter-output)
- [Tuning Procedure](#tuning-procedure)
- [EEPROM Storage](#eeprom-storage)
- [Troubleshooting](#troubleshooting)
- [Performance Notes](#performance-notes)

## Overview

The Arduino Uno version provides a complete development platform for tuning and testing BLDC motor PID control. It includes potentiometer tuning, comprehensive serial commands, EEPROM storage, and real-time monitoring.

### Key Features

- **Potentiometer Tuning**: 5 potentiometers for real-time PID parameter adjustment
- **Serial Command Interface**: Complete command set for parameter tuning and monitoring
- **Serial Plotter Integration**: Real-time visualization of control performance
- **EEPROM Parameter Storage**: Persistent settings across power cycles
- **Modular Configuration**: All settings centralized in `config.h`
- **Debug Mode**: Optional detailed logging for development
- **Three Operating Modes**: Production, Potentiometer Tuning, and Serial Tuning

## Quick Start

1. **Hardware Setup**: Connect components according to pin assignments below
2. **Upload Code**: Load `arduino_uno.ino` to Arduino Uno
3. **Configure**: Modify `config.h` for your specific setup
4. **Tune Parameters**: Use potentiometers or serial commands
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
- 5x 10kΩ potentiometers
- Breadboard and jumper wires

### Pin Connections

| Component | Arduino Pin | Description |
|-----------|-------------|-------------|
| BLDC Hall Sensor | Digital Pin 2 | Any Hall wire from motor (interrupt pin) - provides 2 pulses per electrical revolution |
| PWM Output | Digital Pin 9 | PWM signal to ESC |
| Mode Switch | Digital Pin 3 | LOW = Potentiometer mode, HIGH = Production/Serial mode |
| Target RPM Pot | Analog A0 | Sets target RPM (0-3000 RPM) |
| Kp Pot | Analog A1 | Proportional gain (0-2.0) |
| Ki Pot | Analog A2 | Integral gain (0-1.0) |
| Kd Pot | Analog A3 | Derivative gain (0-0.1) |
| Pulses/Rev Pot | Analog A4 | Pulses per revolution (1-100) |

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
#define POT_TARGET_RPM      A0  // Target RPM potentiometer
#define POT_KP              A1  // Proportional gain pot
#define POT_KI              A2  // Integral gain pot
#define POT_KD              A3  // Derivative gain pot
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
#define EMERGENCY_STOP_ENABLED     true   // Emergency stop on motor faults
#define EMERGENCY_STOP_TIMEOUT_MS  5000  // Emergency stop timeout (5 seconds)
```

## Operating Modes

The Arduino Uno version supports three operating modes selected by the mode switch (Digital Pin 3):

### Production Mode (Default)
- **Mode Switch**: HIGH or floating
- **Behavior**: Uses pre-configured PID parameters for stable operation
- **Features**: Ignores potentiometers and most serial commands
- **Use Case**: Final production deployment

### Potentiometer Tuning Mode
- **Mode Switch**: LOW (connected to GND)
- **Behavior**: Real-time parameter adjustment via 5 potentiometers
- **Monitoring**: Serial Plotter shows live control response
- **Use Case**: Initial PID tuning and testing

### Serial Tuning Mode
- **Mode Switch**: HIGH or floating
- **Activation**: Send `MODE SERIAL` command via Serial Monitor
- **Behavior**: Full serial command interface for parameter adjustment
- **Features**: EEPROM storage and detailed monitoring
- **Use Case**: Precise tuning and parameter optimization

## Serial Commands

Available when in Serial Tuning Mode (115200 baud):

### Parameter Commands
- `SET TARGET <rpm>` - Set target RPM (0-5000)
- `SET KP <value>` - Set proportional gain (0-10.0)
- `SET KI <value>` - Set integral gain (0-5.0)
- `SET KD <value>` - Set derivative gain (0-1.0)
- `SET PULSES <value>` - Set pulses per revolution (1-100)

### Mode Commands
- `MODE PRODUCTION` - Switch to production mode
- `MODE SERIAL` - Switch to serial tuning mode

### Monitoring Commands
- `GET PARAMS` - Display current parameters and status
- `SAVE` - Save current parameters to EEPROM
- `LOAD` - Load parameters from EEPROM
- `RESET INTEGRAL` - Reset integral term to zero
- `HELP` - Display available commands

## Serial Plotter Output

The system outputs four comma-separated values for Serial Plotter visualization:
```
Target,Current,Error,PID_Output
```

- **Target**: Desired RPM setpoint
- **Current**: Measured motor RPM
- **Error**: Difference between target and current
- **PID_Output**: Computed PID control value

### Plotter Setup
1. Open Arduino IDE → Tools → Serial Plotter
2. Set baud rate to 115200
3. Four traces will display real-time control performance

## Tuning Procedure

### Potentiometer Tuning
1. Set mode switch to LOW (potentiometer mode)
2. Open Serial Plotter (115200 baud)
3. Adjust potentiometers while monitoring response:
   - **Pot A0 (Target RPM)**: Set desired speed (start with 1440)
   - **Pot A1 (Kp)**: Start low (0.1), increase until oscillations start
   - **Pot A2 (Ki)**: Add small amount to eliminate steady-state error
   - **Pot A3 (Kd)**: Add minimal damping for stability
   - **Pot A4 (Pulses/Rev)**: Set pulses per revolution for your motor (usually 6 for 3-Hall BLDC)
4. Record optimal potentiometer positions
5. Transfer values to production constants in `config.h`

### Serial Tuning
1. Open Serial Monitor (115200 baud)
2. Send `MODE SERIAL` command
3. Send `HELP` for available commands
4. Use `SET` commands to adjust parameters:
   ```bash
   SET KP 0.5
   SET KI 0.1
   SET KD 0.01
   ```
5. Monitor with `GET PARAMS` and Serial Plotter
6. Save optimal parameters with `SAVE` command

## EEPROM Storage

Parameters are automatically saved to EEPROM addresses:
- Address 0: Target RPM (4 bytes)
- Address 4: Kp gain (4 bytes)
- Address 8: Ki gain (4 bytes)
- Address 12: Kd gain (4 bytes)
- Address 16: Pulses per revolution (4 bytes)

Parameters are validated on load and revert to defaults if corrupted.

## Troubleshooting

### Motor Not Starting
- Verify PWM output pin (Digital Pin 9) connected to ESC signal input
- Check ESC power supply and motor connections
- Confirm Hall sensor providing pulses (monitor with Serial Plotter)
- Verify PID parameters not set to extreme values

### No RPM Reading
- Confirm Hall sensor wire connected to Digital Pin 2
- Check sensor power (5V) and ground connections
- Verify `PULSES_PER_REV` matches your motor (usually 6 for 3-Hall)
- Test with oscilloscope on sensor pin for pulse signals

### Unstable Control
- Reduce Kp gain first, then adjust Ki and Kd
- Check for electrical noise on sensor lines
- Verify proper motor/ESC grounding
- Ensure clean power supplies for Arduino and motor

### Serial Communication Issues
- Verify baud rate set to 115200
- Check Serial Monitor line ending settings
- Commands must be in uppercase
- Use Serial Plotter for graphical monitoring

## Performance Notes

- **Memory Usage**: ~93% RAM usage (normal for feature-rich Arduino sketch)
- **Control Frequency**: 100Hz main loop, 10Hz RPM calculation
- **PWM Frequency**: ~490Hz (Timer0 with prescaler 8)
- **Interrupt Response**: Hall sensor debounce filter prevents false triggers
- **EEPROM Wear**: Parameters saved only on explicit SAVE command
