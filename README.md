# BLDC Motor PID Controller

A robust PID control system for maintaining a BLDC motor at exactly 1440 RPM, with implementations for both development (Arduino Uno) and production (ATTiny85) deployment.

## Table of Contents

- [Project Structure](#project-structure)
- [Overview](#overview)
- [Implementations](#implementations)
  - [Arduino Uno Version](#arduino-uno-version-arduino_uno)
  - [ATTiny85 Version](#attiny85-version-attiny85)
- [Quick Start](#quick-start)
- [Hardware Requirements](#hardware-requirements)
- [Software Architecture](#software-architecture)
- [Operating Modes](#operating-modes)
- [Tuning Procedure](#tuning-procedure)
- [Configuration Parameters](#configuration-parameters)
- [Serial Commands](#serial-commands)
- [Serial Plotter Output](#serial-plotter-output)
- [Safety Features](#safety-features)
- [Usage Instructions](#usage-instructions)
- [Troubleshooting](#troubleshooting)
- [Performance Optimization](#performance-optimization)
- [Future Enhancements](#future-enhancements)
- [Contributing](#contributing)
- [License](#license)

## Project Structure

```
BLDC_PID_Controller/
├── arduino_uno/                 # Development implementation
│   ├── arduino_uno.ino          # Arduino Uno code with serial tuning
│   ├── hardware_schematic.txt   # Hardware setup guide
│   └── README.md               # Detailed Arduino Uno documentation
├── attiny85/                    # Production implementation
│   ├── attiny85.ino            # Production ATTiny85 code
│   ├── README.md               # ATTiny85 documentation
│   └── ATTiny85_hardware_schematic.txt # ATTiny85 hardware setup
├── LICENSE                     # MIT License
└── README.md                   # This overview file
```

## Overview

This Arduino-based controller implements a PID (Proportional-Integral-Derivative) algorithm to precisely control BLDC motor speed. The system maintains exact RPM even under varying load conditions through:

- **PID Control**: Proportional, Integral, and Derivative terms for optimal speed regulation
- **Anti-Windup Protection**: Prevents integral windup during stall or high-load conditions
- **Three Mode Operation**: Production mode for stable operation, potentiometer tuning for hardware adjustment, and serial tuning for software-based parameter optimization
- **Real-time Feedback**: RPM sensor integration for accurate speed measurement
- **Flexible Tuning**: Four potentiometers for hardware tuning or serial commands for software tuning
- **Parameter Persistence**: EEPROM storage for saving and loading tuned PID parameters
- **Multi-Platform Support**: Arduino Uno for development and ATTiny85 for production deployment

## Implementations

### Arduino Uno Version (`arduino_uno/`)
**Best for**: Development, testing, and tuning
- Full serial command interface for real-time PID parameter adjustment
- EEPROM storage for parameter persistence
- Serial Plotter support for monitoring
- Optional potentiometer tuning
- More memory for debugging features

### ATTiny85 Version (`attiny85/`)
**Best for**: Production deployment
- Minimal resource usage (31% flash, 8% RAM)
- No external dependencies
- Optimized for power efficiency
- Pre-tuned PID parameters for stable operation
- Smaller footprint, lower cost

## Quick Start

1. **Choose your platform:**
   - Open `arduino_uno/arduino_uno.ino` for development
   - Open `attiny85/attiny85.ino` for production

2. **Follow the platform-specific README:**
   - `arduino_uno/README.md` for Arduino Uno setup
   - `attiny85/README.md` for ATTiny85 setup

3. **Hardware setup:** Refer to the schematic files in each platform folder

## Hardware Requirements

### Required Components
- Arduino board (Uno, Mega, or similar) or ATtiny85 microcontroller
- BLDC motor with Electronic Speed Controller (ESC)
- BLDC motor Hall sensors (built into most 3-phase BLDC motors)
- SPDT switch or jumper (mode selection, Arduino version only)
- Power supply suitable for motor and microcontroller

### Optional Components (for Hardware Tuning)
- 4x 10kΩ potentiometers (for potentiometer-based tuning mode)

### Pin Connections

#### Arduino Uno Version
| Component | Arduino Pin | Description |
|-----------|-------------|-------------|
| BLDC Hall Sensor | Digital Pin 2 | Any Hall wire from BLDC motor (interrupt-capable pin) |
| PWM Output | Digital Pin 9 | PWM signal to ESC |
| Mode Switch | Digital Pin 3 | LOW = Potentiometer tuning mode, HIGH = Production/Serial mode |
| Target RPM Pot | Analog A0 | Sets target RPM (0-3000 RPM) - Optional |
| Kp Pot | Analog A1 | Proportional gain (0-2.0) - Optional |
| Ki Pot | Analog A2 | Integral gain (0-1.0) - Optional |
| Kd Pot | Analog A3 | Derivative gain (0-0.1) - Optional |

#### ATtiny85 Version
| Component | ATtiny85 Physical Pin | Description |
|-----------|----------------------|-------------|
| BLDC Hall Sensor | Pin 2 (PB3) | Any Hall wire from BLDC motor (interrupt-capable pin) |
| PWM Output | Pin 5 (PB0) | PWM signal to ESC |

### BLDC Hall Sensor Wiring

Most 3-phase BLDC motors have three built-in Hall effect sensors that provide 6 pulses per revolution (one pulse per 60° of rotation). Connect **any one** of the three Hall sensor wires to the microcontroller input pin.

#### Wiring Diagram:
```
BLDC Motor Hall Sensors → Controller/Microcontroller → ESC → Motor Power
     |                        |                        |
     |                        |                        |
   Hall A ──────────────┐     |                        |
   Hall B ──────────────┼─────┼─── Digital Pin 2 ──────┘
   Hall C ──────────────┘     |       (Arduino)
                              |       Pin 2 (PB3)
                              |       (ATtiny85)
                              |
                              └───── PWM Pin 9 ────── ESC Signal Input
                                     (Arduino)
                                     Pin 5 (PB0)
                                     (ATtiny85)
```

#### Key Points:
- **Any Hall wire works**: Connect Hall A, B, or C to the sensor input pin
- **No isolation needed**: The controller and BLDC motor can safely share Hall wires
- **6 pulses per revolution**: Code is configured for 3-Hall BLDC motors
- **Power sharing**: Hall sensors typically operate at 5V, same as microcontroller
- **Common ground**: Ensure all components share a common ground connection

## Software Architecture

### Control Loop
- **Frequency**: 100 Hz (10ms cycle time)
- **RPM Calculation**: Updated every 100ms for stability
- **PID Computation**: Calculated each control cycle
- **PWM Output**: Updated immediately after PID computation
- **Serial Command Processing**: Asynchronous command parsing and execution

### PID Algorithm
```
error = target_RPM - current_RPM
proportional = Kp × error
integral += Ki × error (with anti-windup clamping)
derivative = Kd × (error - previous_error)
output = proportional + integral + derivative
```

### Anti-Windup Protection
- Integral term clamped between -100 and +100
- Prevents runaway during motor stall or maximum load
- Maintains system stability under adverse conditions

## Operating Modes

### Production Mode (Default)
- Uses hardcoded optimal PID parameters
- Target RPM: 1440 RPM
- Kp: 0.5, Ki: 0.1, Kd: 0.01
- Ignores potentiometer inputs and serial commands
- Stable, predictable operation

### Potentiometer Tuning Mode
- Activated when mode switch is LOW
- Real-time parameter adjustment via potentiometers
- Serial Plotter visualization
- Live system response monitoring

### Serial Tuning Mode
- Activated by sending "MODE SERIAL" command via serial
- Real-time parameter adjustment via serial commands
- Interactive command interface with immediate feedback
- Parameter persistence through EEPROM storage

## Tuning Procedure

### Potentiometer Tuning

1. **Set Mode Switch to Tuning**: Connect mode pin to GND
2. **Open Serial Plotter**: Tools → Serial Plotter in Arduino IDE
3. **Adjust Parameters**:
   - **Pot 1 (Target RPM)**: Set to desired speed (start with 1440)
   - **Pot 2 (Kp)**: Start low (0.1), increase until oscillations begin
   - **Pot 3 (Ki)**: Add small amount to eliminate steady-state error
   - **Pot 4 (Kd)**: Add minimal damping for stability
4. **Monitor Response**: Observe target vs current RPM in Serial Plotter
5. **Record Optimal Values**: Note the potentiometer settings that give best response
6. **Update Production Constants**: Replace default values in code

### Serial Tuning

1. **Open Serial Monitor**: Tools → Serial Monitor in Arduino IDE (115200 baud)
2. **Enter Serial Mode**: Type `MODE SERIAL` and press Enter
3. **View Available Commands**: Type `HELP` and press Enter
4. **Set Parameters**: Use commands like:
   - `SET TARGET 1440`
   - `SET KP 0.5`
   - `SET KI 0.1`
   - `SET KD 0.01`
5. **Monitor Response**: Use `GET PARAMS` to view current values
6. **Save Parameters**: Type `SAVE` to store parameters in EEPROM
7. **Test Performance**: Monitor motor response and adjust as needed

## Configuration Parameters

### Control Parameters
```cpp
#define CONTROL_LOOP_HZ     100     // Control loop frequency
#define PULSES_PER_REV      6       // Sensor pulses per revolution (6 for 3-Hall BLDC motors)
#define RPM_CALC_INTERVAL   100     // RPM update interval (ms)
#define SERIAL_BUFFER_SIZE  64      // Serial command buffer size
```

### EEPROM Storage Addresses
```cpp
#define EEPROM_TARGET_RPM_ADDR 0    // Target RPM storage address
#define EEPROM_KP_ADDR         4    // Kp gain storage address
#define EEPROM_KI_ADDR         8    // Ki gain storage address
#define EEPROM_KD_ADDR         12   // Kd gain storage address
```

### PID Limits
```cpp
#define PID_OUTPUT_MIN      -255    // Minimum PID output
#define PID_OUTPUT_MAX      255     // Maximum PID output
#define INTEGRAL_WINDUP_MIN -100    // Anti-windup integral minimum
#define INTEGRAL_WINDUP_MAX 100     // Anti-windup integral maximum
```

### Production Mode Defaults
```cpp
#define PRODUCTION_TARGET_RPM 1440.0
#define PRODUCTION_KP         0.5
#define PRODUCTION_KI         0.1
#define PRODUCTION_KD         0.01
```

## Serial Commands

The system supports interactive serial commands for parameter tuning and monitoring:

### Parameter Setting
- `SET TARGET <rpm>` - Set target RPM (0-5000)
- `SET KP <value>` - Set proportional gain (0-10.0)
- `SET KI <value>` - Set integral gain (0-5.0)
- `SET KD <value>` - Set derivative gain (0-1.0)

### Mode Control
- `MODE PRODUCTION` - Switch to production mode
- `MODE SERIAL` - Switch to serial tuning mode

### Monitoring and Storage
- `GET PARAMS` - Display current parameters and status
- `SAVE` - Save current parameters to EEPROM
- `LOAD` - Load parameters from EEPROM
- `RESET INTEGRAL` - Reset integral term to zero

### Information
- `HELP` - Display available commands

## Serial Plotter Output

The system outputs four comma-separated values for visualization:
- `Target`: Desired RPM
- `Current`: Measured RPM
- `Error`: Difference between target and current
- `PID_Output`: Computed PID control value

Note: Serial Plotter output is available in all operating modes.

## Safety Features

- **Output Clamping**: PID output constrained to safe PWM range
- **Integral Windup Protection**: Prevents integrator runaway
- **Startup Delay**: 1-second initialization period
- **Pull-up Resistors**: RPM sensor pin configured with internal pull-up
- **EEPROM Validation**: Parameters loaded from EEPROM are validated for reasonable ranges
- **Command Validation**: Serial commands are parsed and validated before execution

## Usage Instructions

1. **Hardware Setup**: Connect all components according to pin assignments
2. **Upload Code**: Load `arduino_uno.ino` to Arduino
3. **Initial Testing**: Run in production mode first
4. **Tuning Options**:
   - **Potentiometer Tuning**: Set mode switch LOW and adjust potentiometers
   - **Serial Tuning**: Open Serial Monitor (115200 baud) and use commands
5. **Save Parameters**: Use `SAVE` command or update constants in code
6. **Production**: Switch back to production mode for stable operation

## Troubleshooting

### Common Issues

**Motor Not Starting**
- Check ESC calibration and power connections
- Verify PWM output pin connection
- Confirm RPM sensor is providing pulses

**Unstable Control**
- Reduce Kp gain
- Check for electrical noise on sensor lines
- Verify proper grounding

**No RPM Reading**
- Confirm sensor type and pulse count configuration
- Check interrupt pin connection
- Verify sensor power and signal levels

**Serial Plotter Issues**
- Ensure baud rate matches (115200)
- Check Arduino serial connection
- Verify no other serial devices are conflicting

**Serial Command Issues**
- Commands must end with newline character
- Check Serial Monitor line ending settings
- Ensure commands are in uppercase
- Use HELP command to verify available commands

## Performance Optimization

### PID Tuning Guidelines
1. **Start with P only**: Set Ki=0, Kd=0, tune Kp for stability
2. **Add Integral**: Slowly increase Ki to eliminate steady-state error
3. **Add Derivative**: Small Kd values for improved damping
4. **Fine-tune**: Make small adjustments and observe response

### Hardware Considerations
- Use shielded cables for sensor connections
- Separate power supplies for logic and motor power
- Add RC filters on PWM output if ESC is sensitive to noise

## Future Enhancements

- Multiple PID profiles for different operating conditions
- Temperature compensation
- Advanced filtering for noisy RPM measurements
- CAN bus integration for multi-motor systems
- Web-based tuning interface
- Data logging for performance analysis

## Contributing

Contributions are welcome! Please submit issues and pull requests on GitHub.

## License

This project is released under the MIT License.
