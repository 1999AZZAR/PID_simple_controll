# BLDC Motor PID Controller

A robust PID control system for maintaining a BLDC motor at exactly 1440 RPM, with implementations for both development (Arduino Uno) and production (ATTiny85) deployment.

## Table of Contents

- [Project Structure](#project-structure)
- [Overview](#overview)
- [Implementations](#implementations)
  - [Arduino Uno Version](#arduino-uno-version-arduino_uno)
  - [ATTiny85 Version](#attiny85-version-attiny85)
- [Automated ZIP Creation](#automated-zip-creation)
- [Quick Start](#quick-start)
- [Hardware Requirements](#hardware-requirements)
- [Software Architecture](#software-architecture)
- [Operating Modes](#operating-modes)
- [Tuning Procedure](#tuning-procedure)
- [Configuration Parameters](#configuration-parameters)
- [Serial Plotter Output](#serial-plotter-output)
- [Safety Features](#safety-features)
- [Usage Instructions](#usage-instructions)
- [Troubleshooting](#troubleshooting)
- [Mitigation Guide](mitigation.md)
- [Performance Optimization](#performance-optimization)
- [Future Enhancements](#future-enhancements)
- [Contributing](#contributing)
- [License](#license)

## Project Structure

```
BLDC_PID_Controller/
├── arduino_uno/                 # Development implementation
│   ├── arduino_uno.ino          # Arduino Uno code with serial tuning
│   ├── config.h                 # Configuration header with all settings
│   ├── hardware_schematic.md   # Hardware setup guide
│   └── README.md               # Detailed Arduino Uno documentation
├── attiny85/                    # Production implementation
│   ├── attiny85.ino            # Production ATTiny85 code
│   ├── config.h                 # ATTiny85 configuration
│   ├── README.md               # ATTiny85 documentation
│   └── ATTiny85_hardware_schematic.md # ATTiny85 hardware setup
├── assets/                      # Documentation assets
│   ├── 42BLF.pdf               # 42BLF motor datasheet
│   ├── High-level component diagram.png  # System architecture
│   ├── Main control flow.png    # Control algorithm flowchart
│   └── Serial command sequence.png # Serial interface diagram
├── CONTRIBUTING.md             # Contribution guidelines
├── howto.md                    # Detailed assembly guide
├── mitigation.md               # Comprehensive troubleshooting guide
├── LICENSE                     # MIT License
└── README.md                   # This overview file
```

## Overview

This Arduino-based controller implements a PID (Proportional-Integral-Derivative) algorithm to precisely control BLDC motor speed. The system maintains exact RPM even under varying load conditions through:

- **Motor Compatibility**: Designed for 3-Hall BLDC motors (such as 42BLF20-22.0223 or equivalent) - see `assets/42BLF.pdf` for complete specifications
- **Hall Sensor Integration**: Direct connection to motor Hall sensors (6 pulses per electrical revolution)
- **PID Control**: Proportional, Integral, and Derivative terms for optimal speed regulation
- **Anti-Windup Protection**: Prevents integral windup during stall or high-load conditions
- **Two Mode Operation**: Production mode (fixed 1440 RPM) and potentiometer tuning for hardware adjustment
- **Real-time Feedback**: Hall sensor RPM measurement for accurate closed-loop control
- **Flexible Tuning**: Four potentiometers for hardware tuning (PPR, Kp, Ki, Kd)
- **Parameter Persistence**: EEPROM storage for saving and loading tuned PID parameters
- **Multi-Platform Support**: Arduino Uno for development and ATTiny85 for production deployment

## Documentation Assets

The `assets/` folder contains visual documentation and reference materials:

- **`42BLF.pdf`**: Complete motor datasheet with specifications, dimensions, and electrical characteristics
- **`High-level component diagram.png`**: System architecture showing all major components and connections
- **`Main control flow.png`**: Flowchart of the PID control algorithm and system operation
- **`Serial command sequence.png`**: Visual guide to serial command interactions and responses

These diagrams provide visual reference for:
- Hardware assembly and component identification
- Understanding the control flow and algorithm logic
- Serial communication protocol and command sequences
- System integration and troubleshooting

## System Architecture Overview

![High-level Component Diagram](assets/High-level%20component%20diagram.png)

*Figure 1: High-level system architecture showing Arduino Uno, BLDC motor, ESC, and sensor connections*

## Implementations

### Arduino Uno Version (`arduino_uno/`)

**Best for**: Simplified development and testing

- Streamlined design with fixed 1440 RPM target for maximum reliability
- Serial Plotter monitoring with 7 parameters (Target, Current, Error, PID_Output, Kp, Ki, Kd, PPR)
- Two operating modes: Production (fixed parameters) and Potentiometer tuning
- No serial commands or interactive tuning
- 4 potentiometers: PPR (A0), Kp (A1), Ki (A2), Kd (A3)
- No EEPROM storage or watchdog timer
- Optimized memory usage: 21% flash, 18% RAM (highly stable)
- Clean, maintainable codebase

### ATTiny85 Version (`attiny85/`)

**Best for**: Production deployment

- Minimal resource usage (77% flash, 38% RAM)
- **Core PID Control Only**: Essential motor control with integer math optimization
- **2-Pin Operation**: Hall sensor input + PWM output (ultra-minimal)
- No external dependencies
- Fits in ATTiny85's 2KB flash limit
- Pre-tuned PID parameters hardcoded for reliability
- Smaller footprint, lower cost (~$2 vs $20)
- Production-ready for cost-sensitive applications

## Automated Build & Packaging

This repository uses GitHub Actions for fully automated compilation and packaging:

### Primary Workflow: **Compile Arduino Sketches**
- **One-Click Solution**: Compiles both platforms + creates complete ZIP
- **Generates** ready-to-flash `.hex` files
- **Creates** `ddmmyyyy_reponame.zip` with everything included
- **Attaches** ZIP to GitHub releases automatically

### What You Get
- **Source Code**: All Arduino sketches and configurations
- **Compiled Firmware**: Flash-ready `.hex` files for both platforms
- **Documentation**: Complete setup guides and troubleshooting
- **Memory Reports**: Flash/RAM usage statistics

### Example Output
- `25112025_PID_simple_controll.zip` (November 25, 2025)

### Workflow Triggers
- **Push to main/master**: Auto-creates daily release with complete package
- **Version Tags**: Creates versioned releases on tag pushes
- **Manual**: Actions → "Compile Arduino Sketches" → Run workflow
- **Pull Request**: Validates compilation without creating releases

### Output Locations
- **ZIP Downloads**: Actions → Compile Arduino Sketches → `complete-package` artifact
- **Daily Releases**: Automatic releases created on main branch pushes
- **Release Assets**: Complete packages automatically attached

### Automatic Release System
- **Daily Releases**: Every push creates a dated release (e.g., `25112025`)
- **Complete Packages**: Source code + compiled firmware included
- **No Manual Work**: Fully automated CI/CD pipeline
- **Version Control**: Date-based versioning for easy tracking

### ZIP Contents
```
25112025_PID_simple_controll.zip/
├── arduino_uno/
│   ├── arduino_uno.ino        # Source code
│   ├── compiled/              # ← Ready-to-flash firmware
│   │   ├── arduino_uno.ino.hex
│   │   └── arduino_uno.ino.elf
│   └── README.md
├── attiny85/
│   ├── attiny85.ino           # Source code
│   ├── compiled/              # ← Ready-to-flash firmware
│   │   ├── attiny85.ino.hex
│   │   └── attiny85.ino.elf
│   └── README.md
├── assets/
├── README.md
└── ...
```

## Platform Comparison

| Feature | Arduino Uno (Simplified) | ATTiny85 (Production) |
|---------|---------------------------|----------------------|
| **Purpose** | Clean development and testing | Production deployment |
| **Target RPM** | Fixed at 1440 RPM | Fixed at 1440 RPM |
| **Flash Usage** | 6,968 bytes (21%) | 1,586 bytes (77%) |
| **RAM Usage** | 372 bytes (18%) | 49 bytes (38%) |
| **Efficiency Ratio** | **4.4x smaller than full-featured** | **11.7x smaller flash, 7.6x less RAM** |
| **Total Flash** | 32,256 bytes | 2,048 bytes |
| **Total RAM** | 2,048 bytes | 128 bytes |
| **Pin Count** | 7 pins used | **2 pins only** |
| **Cost** | ~$20 | ~$2 |
| **Tuning Interface** | 4 potentiometers (PPR, Kp, Ki, Kd) | None (pre-tuned) |
| **Safety Features** | Emergency stop, anti-windup | Watchdog timer |
| **EEPROM** | None | None |
| **Serial Output** | 7-parameter monitoring | None |
| **Development Time** | Quick setup, stable operation | Deploy & forget |
| **Power Efficiency** | Standard | Optimized |
| **Reliability Focus** | Maximum stability | Cost-effective |

**Key Takeaway**: Arduino Uno = Development platform with full features. ATTiny85 = Minimal production controller focused on cost and size.

## Quick Start

1. **Choose your platform:**

   - Open `arduino_uno/arduino_uno.ino` for development
   - Open `attiny85/attiny85.ino` for production
2. **Review documentation:**

   - `howto.md` - Detailed step-by-step assembly guide
   - `assets/` - Visual diagrams and motor datasheet
3. **Follow the platform-specific README:**

   - `arduino_uno/README.md` for Arduino Uno setup
   - `attiny85/README.md` for ATTiny85 setup
4. **Hardware setup:** Refer to the schematic files in each platform folder

## Hardware Requirements

### Required Components

- Arduino board (Uno, Mega, or similar) or ATtiny85 microcontroller
- **3-Hall BLDC motor** (such as 42BLF20-22.0223 or equivalent with built-in Hall sensors) - see `assets/42BLF.pdf` datasheet for specifications
- **BLDC motor controller (ESC)** compatible with the specific motor model - see PWM/ESC Interface section below
- **BLDC motor Hall sensors** (built into the motor - any Hall wire A/B/C can be used)
- SPDT switch or jumper (mode selection, Arduino version only)
- Power supply suitable for motor and microcontroller (5V for Hall sensor compatibility)

### Optional Components (for Hardware Tuning)

- 5x 10kΩ potentiometers (for potentiometer-based tuning mode)

### Pin Connections

#### Arduino Uno Version

| Component        | Arduino Pin   | Description                                                    |
| ---------------- | ------------- | -------------------------------------------------------------- |
| BLDC Hall Sensor | Digital Pin 2 | Any Hall wire from BLDC motor (interrupt-capable pin)          |
| PWM Output       | Digital Pin 9 | PWM signal to ESC                                              |
| Mode Switch      | Digital Pin 3 | LOW = Potentiometer tuning mode, HIGH = Production mode        |
| PPR Pot          | Analog A0     | Pulses per revolution (1-100) - Optional                       |
| Kp Pot           | Analog A1     | Proportional gain (0-2.0) - Optional                           |
| Ki Pot           | Analog A2     | Integral gain (0-1.0) - Optional                               |
| Kd Pot           | Analog A3     | Derivative gain (0-0.1) - Optional                             |

#### ATtiny85 Version

| Component        | ATtiny85 Physical Pin | Description                                           |
| ---------------- | --------------------- | ----------------------------------------------------- |
| BLDC Hall Sensor | Pin 2 (PB3)           | Any Hall wire from BLDC motor (interrupt-capable pin) |
| PWM Output       | Pin 5 (PB0)           | PWM signal to ESC                                     |

### BLDC Hall Sensor Wiring

This controller is designed for **3-Hall BLDC motors** such as the 42BLF20-22.0223 or equivalent motors with built-in Hall effect sensors (see `assets/42BLF.pdf` for complete motor specifications). These motors have three Hall sensors (Hall A, Hall B, Hall C) that provide 6 pulses per electrical revolution (one pulse per 60° of rotation). Connect **any one** of the three Hall sensor wires to the microcontroller input pin.

#### Wiring Diagram:

**Text-based wiring reference:**
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

*See Figure 1 in System Architecture Overview for visual wiring diagram*

#### Key Points:

- **Any Hall wire works**: Connect Hall A, B, or C to the sensor input pin
- **No isolation needed**: The controller and BLDC motor can safely share Hall wires
- **6 pulses per revolution**: Code is configured for 3-Hall BLDC motors
- **Power sharing**: Hall sensors typically operate at 5V, same as microcontroller
- **Common ground**: Ensure all components share a common ground connection

#### Hall Sensor Signal Composition

For optimal performance, the controller can use a composite signal from all three Hall sensors instead of relying on a single Hall sensor wire. This provides more robust position feedback and better motor control.

**Signal Composition Methods:**
1. **Single Hall Wire**: Connect any one Hall sensor (A, B, or C) - provides 2 pulses per electrical revolution
2. **Composite Signal**: Combine all three Hall sensors using OR logic - provides 6 pulses per electrical revolution
3. **Sensor Selection**: Use the Hall sensor that provides the most stable signal for your specific motor

**Composite Signal Benefits:**
- **Higher Resolution**: 6 pulses vs 2 pulses per electrical revolution
- **Better Reliability**: Redundant position information from three sensors
- **Improved Control**: More frequent position updates for smoother motor control

**Implementation:**
```cpp
// Single Hall sensor (current implementation)
attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, RISING);

// Composite signal would require external OR gate or microcontroller OR logic
// Each Hall sensor contributes to the composite pulse train
```

#### Hall Sensor Edge Configuration

BLDC motor Hall sensors can be either push-pull or open-collector outputs:

- **Push-pull Hall sensors**: Output both HIGH and LOW states actively. Use RISING edge detection for reliable triggering.
- **Open-collector Hall sensors**: Require pull-up resistors and only actively pull LOW. Use FALLING edge detection to avoid false triggers from slow rising edges.

**Current Configuration:**
- Arduino Uno: Uses RISING edge detection with INPUT_PULLUP (assumes push-pull or properly pulled-up open-collector sensors)
- ATtiny85: Uses FALLING edge detection with internal pull-up (configured for open-collector sensors)

**Verification Steps:**
1. Check motor datasheet for Hall sensor output type
2. Monitor Hall sensor signal with oscilloscope during motor rotation
3. Adjust edge detection (RISING vs FALLING) based on observed signal characteristics
4. Ensure pull-up resistors are appropriate for your sensor type

### PWM/ESC Interface Requirements

This controller outputs standard Arduino PWM signals (0-255 duty cycle) to control BLDC motor speed through an Electronic Speed Controller (ESC). The PWM frequency is approximately 490Hz (Arduino) or 1kHz (ATtiny85).

#### PWM Signal Specifications

- **Signal Type**: Digital PWM (Pulse Width Modulation)
- **Frequency**: ~490Hz (Arduino Uno) or ~1kHz (ATtiny85)
- **Duty Cycle Range**: 0-100% (0-255 in 8-bit resolution)
- **Voltage Level**: 5V logic level
- **Pulse Polarity**: Active HIGH (non-inverted)

#### ESC Compatibility Notes

- **Standard RC ESCs**: Many ESCs expect servo-style pulses (1-2ms pulses at 50Hz). If your ESC requires this format, you will need to modify the code to generate servo-compatible pulses instead of analogWrite() PWM.
- **BLDC ESCs**: Most BLDC motor controllers accept standard PWM signals. Always check your ESC datasheet for signal requirements.
- **Arming Sequence**: Some ESCs require an arming sequence (low throttle for 2+ seconds before operation). The soft-start feature helps with this.
- **Signal Pin**: Connect to the ESC's throttle/signal input pin
- **Power Supply**: Ensure the ESC shares a common ground with the microcontroller

#### Testing PWM Output

To verify PWM compatibility:
1. Connect an oscilloscope or logic analyzer to the PWM output pin
2. Monitor the signal frequency and duty cycle range
3. Compare with your ESC's specifications
4. If incompatible, consider using servo library functions for 50Hz servo pulses

#### PWM vs Servo Pulse Verification Guide

**Critical**: Your ESC may expect either standard PWM signals or RC servo pulses. Using the wrong signal type will result in no motor response or erratic behavior.

##### Method 1: Oscilloscope Analysis (Recommended)
1. **Connect oscilloscope** to PWM output pin (Arduino pin 9, ATtiny85 pin 5)
2. **Observe signal characteristics**:
   - **Frequency**: Arduino Uno outputs ~490Hz, ATtiny85 outputs ~1kHz
   - **Duty cycle**: 0-100% (0-255 in code)
   - **Waveform**: Square wave with varying pulse width

3. **Compare with ESC specifications**:
   - **PWM ESCs** expect: Variable duty cycle (0-100%) at high frequency (1-20kHz)
   - **RC Servo ESCs** expect: Fixed 50Hz frequency with 1-2ms pulse width

##### Method 2: Bench Testing Without Oscilloscope
1. **Set fixed PWM values** and observe motor response:
   ```arduino
   // In setup(), temporarily replace PID output with fixed values:
   analogWrite(PWM_OUTPUT_PIN, 64);   // 25% duty cycle
   delay(2000);
   analogWrite(PWM_OUTPUT_PIN, 128);  // 50% duty cycle
   delay(2000);
   analogWrite(PWM_OUTPUT_PIN, 192);  // 75% duty cycle
   delay(2000);
   ```

2. **Expected behavior**:
   - **PWM ESC**: Motor speed should change smoothly with each duty cycle change
   - **RC Servo ESC**: Motor may not respond or respond erratically

##### Method 3: ESC Documentation Check
- **BLDC ESCs** (like those for quadcopters): Usually PWM input
- **RC Car/Marine ESCs**: Usually servo pulse input (50Hz, 1-2ms)
- **Industrial servo drives**: Check datasheet for "analog input" vs "pulse input"

##### Converting to Servo Pulses (if needed)
If your ESC requires servo pulses, modify the `outputToESC()` function:
```arduino
#include <Servo.h>
Servo escServo;

void setup() {
    escServo.attach(PWM_OUTPUT_PIN);
    // Arming sequence: minimum throttle for 2+ seconds
    escServo.writeMicroseconds(1000);  // 1ms = minimum throttle
    delay(2000);
}

void outputToESC(int pwmValue) {
    // Convert 0-255 PWM to 1000-2000µs servo pulses
    int servoPulse = map(pwmValue, 0, 255, 1000, 2000);
    escServo.writeMicroseconds(servoPulse);
}
```

##### Common Issues
- **No motor response**: Wrong signal type (PWM vs servo)
- **Erratic behavior**: Signal frequency too high/low for ESC
- **Motor runs at full speed only**: ESC expecting servo pulses, getting PWM
- **Noisy operation**: PWM frequency causing interference

## Software Architecture

### Control Loop

- **Frequency**: 100 Hz (10ms cycle time)
- **RPM Calculation**: Updated every 100ms for stability
- **PID Computation**: Calculated each control cycle
- **PWM Output**: Updated immediately after PID computation
- **Serial Command Processing**: Asynchronous command parsing and execution

### PID Algorithm

![Main Control Flow](assets/Main%20control%20flow.png)

*Figure 2: PID control algorithm flowchart showing error calculation, PID computation, and PWM output generation*

```
error = target_RPM - current_RPM              // target_RPM = 1440
proportional = Kp × error                     // Kp = 0.3 (gentle response)
integral += Ki × error (with anti-windup clamping)  // Ki = 0.02 (conservative)
derivative = Kd × (error - previous_error)         // Kd = 0.005 (noise damped)
output = proportional + integral + derivative     // Range: -1000 to +1000 (4x resolution)
```

### Anti-Windup Protection

- Integral term clamped between -100 and +100
- Prevents runaway during motor stall or maximum load
- Maintains system stability under adverse conditions

## Operating Modes

### Production Mode (Default)

- Uses fixed optimal PID parameters
- Target RPM: 1440 RPM
- Kp: 0.5, Ki: 0.1, Kd: 0.01
- Ignores potentiometer inputs
- Stable, predictable operation for production use

### Potentiometer Tuning Mode

- Activated when mode switch pin 3 is connected to GND
- Real-time parameter adjustment via 5 potentiometers (A0-A4)
- Serial Plotter visualization with 7 parameters
- Live system response monitoring for tuning

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

<!-- Serial tuning removed from Arduino Uno version -->

## Configuration Parameters

### Control Parameters

```cpp
#define CONTROL_LOOP_HZ     100     // Control loop frequency
#define PULSES_PER_REV          6   // Default sensor pulses per revolution (6 for 3-Hall BLDC motors like 42BLF20-22.0223 - see assets/42BLF.pdf)
// Note: Arduino Uno version uses runtime variable pulsesPerRev (configurable via serial/EEPROM)
#define RPM_CALC_INTERVAL   100     // RPM update interval (ms)
#define SERIAL_BUFFER_SIZE  64      // Serial command buffer size
```

<!-- EEPROM storage removed from Arduino Uno version -->

### PID Limits

```cpp
#define PID_OUTPUT_MIN      -1000   // Minimum PID output (expanded range)
#define PID_OUTPUT_MAX      1000    // Maximum PID output (expanded range)
#define INTEGRAL_WINDUP_MIN -100    // Anti-windup integral minimum (optimized through testing)
#define INTEGRAL_WINDUP_MAX 100     // Anti-windup integral maximum (optimized through testing)
```

### Production Mode Defaults

```cpp
#define PRODUCTION_TARGET_RPM 1440.0   // Original target RPM
#define PRODUCTION_KP         0.3      // Gentler proportional response
#define PRODUCTION_KI         0.02     // Conservative integral action
#define PRODUCTION_KD         0.005    // Noise-damped derivative
```

<!-- Serial Commands removed from Arduino Uno version -->

## Serial Plotter Output

The Arduino Uno version outputs seven comma-separated values for comprehensive monitoring:

- `Target`: Desired RPM setpoint
- `Current`: Measured motor RPM
- `Error`: Difference between target and current
- `PID_Output`: Computed PID control value
- `Kp`: Current proportional gain
- `Ki`: Current integral gain
- `Kd`: Current derivative gain
- `PPR`: Pulses per revolution setting

Note: Serial Plotter monitoring is available in both operating modes for real-time system observation.

## Safety Features

### Arduino Uno Version (Simplified)
- **Stable Design**: No watchdog timer or emergency stop for maximum reliability
- **Software Protection**: PID output constrained to safe PWM range (0-255)
- **Integral Windup Protection**: Prevents integrator runaway during stall conditions
- **Clean Architecture**: Minimal code for reduced complexity and memory usage

### ATTiny85 Version (Production)
- **Hardware-Level Protection**: 8-second watchdog timer for hang protection
- **Emergency Stop**: Automatic shutdown on motor faults or pulse loss
- **Minimal Footprint**: Core PID control with essential safety features
- **Production Ready**: Reliable operation in resource-constrained environments

## Usage Instructions

1. **Hardware Setup**: Connect components according to Arduino Uno pin assignments
2. **Upload Code**: Load `arduino_uno/arduino_uno.ino` to Arduino Uno
3. **Initial Testing**: System runs in production mode by default
4. **Tuning (Optional)**: Connect mode pin (3) to GND for potentiometer tuning mode
5. **Monitoring**: Open Serial Plotter (115200 baud) to view 7 parameters in real-time
6. **Production Use**: Keep mode pin (3) floating or HIGH for stable operation

## Troubleshooting

**For comprehensive troubleshooting guide with detailed mitigation strategies, see [`mitigation.md`](mitigation.md)**

This document covers advanced issues including RPM measurement discrepancies, power supply conflicts, ATTiny85 programming problems, and PID tuning challenges.

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

We welcome contributions from the community! Please see our [Contributing Guide](CONTRIBUTING.md) for detailed information on:

- Development setup and coding standards
- How to submit pull requests
- Testing procedures and guidelines
- Reporting issues and requesting features

For questions or discussions, please open an issue on GitHub.

## Visual Documentation

The README includes embedded diagrams that provide visual context for the system architecture and operation. These images are displayed inline when viewing this document in compatible markdown viewers (such as GitHub, GitLab, or modern IDEs).

- **Figure 1**: System Architecture - High-level component diagram
- **Figure 2**: Control Flow - PID algorithm flowchart

For additional technical details, refer to the `assets/` folder containing the original high-resolution diagram files.

## License

This project is released under the [MIT License](LICENSE).
