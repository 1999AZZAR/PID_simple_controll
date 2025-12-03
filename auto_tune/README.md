# Arduino Uno BLDC PID Controller - Python Configurable Version

**Advanced PID controller** for BLDC motors with Python GUI configuration and real-time monitoring.

## Table of Contents

- [Overview](#overview)
- [Key Features](#key-features)
- [Quick Start](#quick-start)
- [Installation](#installation)
- [Project Structure](#project-structure)
- [Hardware Setup](#hardware-setup)
- [Software Setup](#software-setup)
- [Usage](#usage)
- [Serial Communication Protocol](#serial-communication-protocol)
- [Troubleshooting](#troubleshooting)
- [Performance Notes](#performance-notes)

## Overview

This advanced version of the Arduino Uno BLDC PID controller replaces potentiometer-based tuning with a comprehensive Python GUI. The system provides real-time parameter adjustment and live performance monitoring.

![img](../assets/python_ui.png "python ui")

### Key Differences from Basic Version

- **No potentiometers required** - All configuration via Python GUI
- **Real-time monitoring** - Live plotting of motor performance
- **Parameter persistence** - Save/load configurations to/from files
- **Advanced visualization** - Multiple real-time graphs and data export

## Key Features

- **Modern PyQt6 Interface**: Professional, responsive GUI with excellent layout management
- **Real-time Parameter Adjustment**: Sliders and spinboxes for precise PID tuning
- **Live Monitoring**: Real-time plotting of motor performance with matplotlib
- **Serial Communication**: Robust bidirectional communication with Arduino
- **Parameter Management**: Save/load configurations to JSON files
- **Data Export**: Export performance data to CSV for analysis
- **Motor Control**: Enable/disable motor, reset controller functions
- **PPR Verification**: Test pulse counting accuracy for correct RPM calculation
- **Professional Styling**: Modern UI with hover effects and visual feedback

## Quick Start

1. **Hardware Setup**: Connect Arduino and motor as described below
2. **Upload Code**: Load `code/code.ino` to Arduino Uno
3. **Install Dependencies**: `pip install -r requirements.txt`
4. **Run GUI**: `python control.py`
5. **Connect**: Select serial port and click "Connect"
6. **Configure**: Adjust parameters using GUI controls
7. **Auto-Tune**: Use automatic PID tuning for optimal performance
8. **Test Connection**: Use built-in serial testing to verify Arduino communication

## Serial Testing Features

The GUI includes integrated serial communication testing:

### Built-in Tests

- **Connection Test**: Verify Arduino connectivity and basic communication
- **Parameter Tests**: Test PID parameter setting commands
- **Motor Control Tests**: Test motor enable/disable functionality
- **Controller Tests**: Test reset and control functions

### Manual Commands

- Send custom commands directly to Arduino
- Real-time log of all communication
- Command history and response monitoring

### Usage

1. Connect to Arduino using the Serial Connection section
2. Click "Test Connection" in the Serial Testing section
3. View results in the log area
4. Use manual command input for custom testing

## Installation

### Arduino Setup

1. Open `code/code.ino` in Arduino IDE
2. Connect Arduino Uno to computer
3. Select correct board and port
4. Upload the sketch

### Python Setup

```bash
# Install dependencies
pip install -r requirements.txt

# Install PyQt6 interface:
pip install PyQt6 matplotlib pyserial numpy
```

### System Requirements

- **Python**: 3.7 or higher
- **Arduino IDE**: Latest version for uploading code
- **Serial Port**: Access to Arduino serial port
- **Operating System**: Windows, macOS, or Linux

### GUI Interface

**PyQt6 Interface** (`control.py`) - **Primary**

- Arduino code located in `code/` folder (Arduino IDE compatible)
- Modern, professional appearance with excellent styling
- Superior layout management - all controls guaranteed visible
- Dual control system: sliders and spinboxes for precise adjustment
- Native OS integration and window management
- Threaded serial communication for better performance
- Advanced auto-tuning with validation and optimization
- **Integrated Serial Testing**: Built-in connection and command testing
- Manual command interface for custom Arduino communication
- Real-time communication logs and status monitoring
- Resizable splitter panels for optimal workspace layout
- Professional styling with hover effects and visual feedback

## Project Structure

```
auto_tune/
├── code/                        # Arduino sketch folder (Arduino IDE compatible)
│   ├── code.ino                 # Arduino sketch with serial communication
│   └── config.h                 # Configuration header
├── control.py                   # Modern PyQt6 GUI application with integrated testing
├── requirements.txt             # Python dependencies
└── README.md                   # This documentation
```

## Hardware Setup

### Required Components

- Arduino Uno (or compatible board)
- 3-Hall BLDC motor (42BLF20-22.0223 or equivalent)
- BLDC motor controller (ESC) compatible with motor
- Any one Hall sensor wire from the BLDC motor

### Pin Connections

| Component        | Arduino Pin   | Description                              |
| ---------------- | ------------- | ---------------------------------------- |
| BLDC Hall Sensor | Digital Pin 2 | Any Hall wire from motor (interrupt pin) |
| PWM Output       | Digital Pin 9 | PWM signal to ESC                        |
| Power/Ground     | 5V/GND        | Power for Hall sensor                    |

### Power Supply Requirements

- **Arduino**: 7-12V via barrel jack or VIN pin
- **Motor/ESC**: Appropriate voltage for your motor (check specifications)
- **Separate Supplies**: Use separate power supplies for logic and motor power

## Software Setup

### Arduino Configuration

The Arduino code is pre-configured for most applications. Key settings in `config.h`:

```cpp
#define DEFAULT_TARGET_RPM 1440.0  // Default target speed
#define DEFAULT_KP         0.25    // Default proportional gain
#define DEFAULT_KI         0.015   // Default integral gain
#define DEFAULT_KD         0.003   // Default derivative gain
#define DEFAULT_PULSES_PER_REV 18  // Hall sensor pulses per revolution
```

### Python GUI Configuration

The GUI automatically detects available serial ports. Default settings:

- **Baud Rate**: 115200 (matches Arduino)
- **Update Rate**: 20 Hz (status updates)
- **Plot Buffer**: 500 data points

## Usage

### Starting the Application

```bash
# Launch the PyQt6 GUI application
python control.py
```

### Connecting to Arduino

1. Select the correct serial port from the dropdown
2. Verify baud rate is 115200
3. Click "Connect"
4. Status should show "Connected" in green

### PPR Configuration

1. Set pulses per revolution (PPR) for your motor's Hall sensor
2. Click "Test PPR" to verify pulse counting accuracy
3. Test counts pulses for 5 seconds and calculates RPM
4. Compare with known motor speed to verify PPR setting
5. Adjust PPR value if RPM calculation is incorrect

### Parameter Adjustment

#### Target RPM

- Use the spinbox to set desired motor speed
- Range: 0-5000 RPM
- Changes take effect immediately

#### PID Gains

- **Kp (Proportional)**: 0-5.0 range, affects response speed
- **Ki (Integral)**: 0-1.0 range, affects steady-state error
- **Kd (Derivative)**: 0-0.1 range, affects stability

#### Pulses Per Revolution

- Set according to your motor's Hall sensor configuration
- Typical values: 6, 12, 18, 24

### Motor Control

- **Enable Motor**: Starts PWM output to ESC
- **Disable Motor**: Stops PWM output (motor coasts)
- **Reset Controller**: Clears integral term and resets soft-start

### Monitoring

The GUI displays four real-time plots:

1. **Motor Speed**: Target vs. current RPM
2. **Control Error**: Difference between target and actual speed
3. **PID Output**: Controller output value
4. **PID Gains**: Current gain values over time

## Serial Communication Protocol

### Commands (Python → Arduino)

```
SET_KP <value>              # Set proportional gain
SET_KI <value>              # Set integral gain
SET_KD <value>              # Set derivative gain
SET_TARGET_RPM <value>      # Set target RPM
SET_PULSES_PER_REV <value>  # Set pulses per revolution
ENABLE_MOTOR <0|1>          # Enable/disable motor
RESET_CONTROLLER            # Reset controller state
GET_STATUS                  # Request status update
FORCE_STATUS                # Force immediate status update
DIAGNOSTICS                 # Run system diagnostics
FORCE_STOP                  # Emergency motor stop
```

### Status Response (Arduino → Python)

```
STATUS:<timestamp>,<target_rpm>,<current_rpm>,<error>,<pid_output>,<kp>,<ki>,<kd>,<ppr>,<motor_enabled>
```

### Data Format

- All values are comma-separated
- Numeric values use decimal format
- Boolean values are 0 or 1
- Commands end with newline (`\n`)

## Troubleshooting

### Connection Issues

**Problem**: Cannot connect to Arduino

- Check serial port selection
- Verify Arduino is powered and programmed
- Try different baud rates
- Check USB cable connection

**Problem**: No data received

- Verify Arduino sketch uploaded correctly
- Check serial monitor in Arduino IDE
- Ensure correct baud rate (115200)

### Motor Control Issues

**Problem**: Motor not starting

- Check PWM output pin connection (Digital Pin 9)
- Verify ESC power and motor connections
- Check Hall sensor connection (Digital Pin 2)
- Use "Enable Motor" button

**Problem**: Unstable control

- Reduce Kp gain first
- Check for electrical noise on sensor lines
- Verify proper motor/ESC grounding
- Ensure clean power supplies

### Auto-Tuning Issues

**Problem**: Auto-tune fails

- Ensure motor is properly connected
- Check that motor can reach target speed
- Verify Hall sensor is providing pulses
- Try manual tuning first

**Problem**: Poor auto-tune results

- Fine-tune manually after auto-tuning
- Adjust target RPM to realistic values
- Check motor/load characteristics

## Performance Notes

### Arduino Performance

- **Memory Usage**: ~25% RAM usage (EEPROM storage adds overhead)
- **Control Frequency**: 100Hz main loop, 20Hz status updates
- **PWM Frequency**: ~490Hz (Timer0 with prescaler 8)
- **Interrupt Response**: Hall sensor debounce prevents false triggers

### Python GUI Performance

- **Update Rate**: 10Hz plot updates, 20Hz data reception
- **Data Buffer**: 500 points maximum (adjustable)
- **CPU Usage**: Minimal when connected, higher during plotting

### Communication Latency

- **Serial Delay**: ~10-20ms round trip
- **GUI Responsiveness**: Immediate local updates, serial-delayed Arduino response
- **Plot Smoothing**: Real-time updates may show minor jitter

## Advanced Features

### Parameter Persistence

Parameters are automatically saved to Arduino EEPROM:

- Survives power cycles
- Manual save/load via GUI
- Export configurations as JSON files

### Data Analysis

- **Real-time Export**: Save plot data to CSV
- **Parameter Logging**: Track gain changes over time
- **Performance Metrics**: Calculate stability, overshoot, settling time

### Custom Tuning

For advanced users, the Ziegler-Nichols gains can be modified:

```python
# Conservative tuning (more stable)
kp_zn = 0.45 * ku
ti = 0.83 * tu
td = 0.125 * tu

# Aggressive tuning (faster response)
kp_zn = 0.8 * ku
ti = 0.3 * tu
td = 0.15 * tu
```

## Version History

- **v1.0** (December 2025): Initial Python GUI release
  - Serial communication with Arduino
  - Real-time parameter adjustment
  - Auto PID tuning (Ziegler-Nichols)
  - Live plotting and monitoring
  - Parameter save/load functionality

## License

This project is released under the MIT License. See the main project LICENSE file for details.

## Contributing

See the main project CONTRIBUTING.md for guidelines.

## Support

For issues and questions:

1. Check the troubleshooting section above
2. Review Arduino serial output for error messages
3. Verify hardware connections
4. Test with known-good parameters first
