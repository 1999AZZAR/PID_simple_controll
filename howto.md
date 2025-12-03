# BLDC Motor PID Controller Assembly Guide

## Table of Contents

- [Introduction](#introduction)
- [Bill of Materials](#bill-of-materials)
- [Software Prerequisites](#software-prerequisites)
- [Arduino Uno Version Assembly](#arduino-uno-version-assembly)
- [ATtiny85 Version Assembly](#attiny85-version-assembly)
- [Software Installation](#software-installation)
- [Initial Testing](#initial-testing)
- [PID Tuning Procedure](#pid-tuning-procedure)
- [Production Deployment](#production-deployment)
- [Troubleshooting](#troubleshooting)
- [Safety Considerations](#safety-considerations)

## Introduction

This guide provides detailed step-by-step instructions for assembling and configuring a PID-controlled BLDC motor system. The controller maintains precise motor speed (DEFAULT_TARGET_RPM) using feedback from the motor's built-in Hall effect sensors.

Two implementation versions are available:
- **Arduino Uno Version**: Full-featured development and tuning platform
- **ATtiny85 Version**: Production-optimized microcontroller with minimal hardware requirements

## Bill of Materials

### Required Components

| Component | Quantity | Specifications | Purpose |
|-----------|----------|----------------|---------|
| Microcontroller | 1 | Arduino Uno R3 or ATtiny85-20PU | Main controller |
| BLDC Motor | 1 | 42BLF20-22.0223 or equivalent 3-Hall BLDC motor | Motor to be controlled |
| BLDC ESC | 1 | Compatible with motor specifications | Motor power controller |
| Power Supply | 1 | 7-12V DC, 2A minimum | Arduino power supply |
| Motor Power Supply | 1 | As required by motor/ESC | Motor power source |
| Hook-up Wire | Various | 22-26 AWG solid core | Electrical connections |
| Jumper Wires | 6-10 | Male-female or male-male | Temporary connections |

### Optional Components (Arduino Uno Tuning)

| Component | Quantity | Specifications | Purpose |
|-----------|----------|----------------|---------|
| Potentiometer | 4 | 10kΩ linear taper rotary | PID parameter adjustment |
| SPDT Switch | 1 | Miniature toggle or slide switch | Mode selection |
| Breadboard | 1 | 400-830 tie points | Circuit prototyping |
| Header Pins | 20 | 0.1" spacing | Breadboard connections |
| Resistor | 1 | 4.7kΩ (optional) | Pull-up for sensor signal |

### Optional Components (Programming)

| Component | Quantity | Specifications | Purpose |
|-----------|----------|----------------|---------|
| Arduino Uno | 1 | Additional for programming ATtiny85 | ISP programmer |
| Capacitor | 1 | 10µF electrolytic | Power supply decoupling |

### Component Sourcing

#### Microcontrollers
- **Arduino Uno**: Available from Arduino.cc, SparkFun, Adafruit, or electronics distributors
- **ATtiny85**: Available from Microchip, Digi-Key, Mouser, or electronics distributors

#### BLDC Motor and ESC
- **42BLF20-22.0223 Motor**: Available from Portescap, Digi-Key, or specialized motor suppliers
- **Compatible ESC**: Must support PWM input (50Hz servo or 1-20kHz PWM). Available from hobby electronics suppliers

#### Electronic Components
- **Potentiometers**: Available from electronics distributors or hobby electronics stores
- **Switches and Connectors**: Available from electronics distributors
- **Wire and Cables**: Available from electronics distributors or hardware stores

### Tools Required

- Soldering iron and solder (for permanent connections)
- Wire strippers
- Multimeter (for testing connections)
- Oscilloscope or logic analyzer (optional, for PWM verification)
- Computer with USB port
- Small screwdriver set

## Software Prerequisites

### Required Software

1. **Arduino IDE** (version 1.8.19 or later)
   - Download from: https://www.arduino.cc/en/software
   - Install on Windows, macOS, or Linux

2. **ATTinyCore** (for ATtiny85 development)
   - Required only if using ATtiny85 version

### Arduino IDE Installation

#### Windows
1. Download the Windows installer from arduino.cc
2. Run the installer executable
3. Follow the installation wizard
4. Launch Arduino IDE to verify installation

#### macOS
1. Download the macOS disk image from arduino.cc
2. Open the downloaded .dmg file
3. Drag Arduino IDE to Applications folder
4. Launch Arduino IDE to verify installation

#### Linux (Ubuntu/Debian)
```bash
# Update package list
sudo apt update

# Install Arduino IDE
sudo apt install arduino

# Launch Arduino IDE to verify
arduino
```

### ATTinyCore Installation (Required for ATtiny85)

1. Open Arduino IDE
2. Go to File → Preferences
3. In "Additional Boards Manager URLs" field, add:
   ```
   https://raw.githubusercontent.com/damellis/attiny/ide-1.6.x-boards-manager/package_damellis_attiny_index.json
   ```
4. Click OK to close Preferences
5. Go to Tools → Board → Boards Manager
6. Search for "attiny"
7. Install "ATTinyCore by Spence Konde"

### Alternative: Arduino CLI

For command-line users or automated builds:

```bash
# Install Arduino CLI
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# Install required cores
arduino-cli core install arduino:avr
arduino-cli core install attiny:avr
```

## Arduino Uno Version Assembly

### Step 1: Prepare Components

1. Verify Arduino Uno board functionality:
   - Connect to computer via USB
   - Confirm power LED illuminates
   - Open Arduino IDE and select Tools → Board → Arduino Uno

2. Prepare potentiometers (if using tuning mode):
   - Verify 10kΩ resistance between outer terminals
   - Confirm linear taper (B-type) potentiometers

3. Prepare SPDT switch for mode selection

### Step 2: Basic Connections

#### Power Connections
1. Connect Arduino Uno to computer via USB cable
2. Connect motor power supply to ESC according to ESC documentation
3. Connect Arduino GND to motor power supply ground (common ground)

#### BLDC Hall Sensor Connection
1. Identify the three Hall sensor wires on the BLDC motor (usually labeled Hall A, Hall B, Hall C)
2. **Option 1 - Single Hall Sensor (Recommended for beginners):**
   - Connect ANY ONE of the Hall wires (A, B, or C) to Arduino digital pin 2
   - This provides 2 pulses per electrical revolution
3. **Option 2 - Composite Hall Sensor (Advanced - better performance):**
   - Use external OR gate or diode circuit to combine all three Hall sensors
   - Connect composite output to Arduino digital pin 2
   - This provides 6 pulses per electrical revolution for smoother control
4. Connect motor Hall sensor power and ground to Arduino 5V and GND pins
   - Most motors share power with the Hall sensors
   - Verify voltage compatibility (should be 5V)

#### ESC PWM Connection
1. Connect Arduino digital pin 9 to ESC signal input
2. Ensure ESC power connections are secure
3. Do not connect motor to ESC yet

### Step 3: Optional Tuning Components

#### Mode Switch
1. Connect one terminal of SPDT switch to Arduino digital pin 3
2. Connect the other terminal to Arduino GND
3. Switch position determines mode:
   - Closed (connected to GND): Potentiometer tuning mode
   - Open (floating): Production/serial tuning mode

#### Potentiometers (4x 10kΩ)
Connect each potentiometer as follows:

1. **PPR Potentiometer** (Pulses per Revolution):
   - Wiper (middle terminal) → Arduino analog pin A0
   - One outer terminal → Arduino GND
   - Other outer terminal → Arduino 5V

2. **Kp Potentiometer** (Proportional Gain):
   - Wiper → Arduino analog pin A1
   - Outer terminals → GND and 5V

3. **Ki Potentiometer** (Integral Gain):
   - Wiper → Arduino analog pin A2
   - Outer terminals → GND and 5V

4. **Kd Potentiometer** (Derivative Gain):
   - Wiper → Arduino analog pin A3
   - Outer terminals → GND and 5V

### Step 4: Wiring Verification

1. Use multimeter to verify:
   - No short circuits between power and ground
   - Correct voltage at Hall sensor connections (5V)
   - Continuity of all signal connections

2. Double-check pin assignments:
   - Hall sensor → Digital pin 2
   - PWM output → Digital pin 9
   - Mode switch → Digital pin 3
   - Potentiometers → Analog pins A0-A3

## ATtiny85 Version Assembly

### Step 1: Prepare ATtiny85

1. Obtain ATtiny85-20PU microcontroller
2. Verify package type (DIP-8 for breadboard use)

### Step 2: Programming Setup

#### Using Arduino as ISP Programmer
1. Connect Arduino Uno as ISP programmer:
   - Arduino pin 10 → ATtiny85 pin 1 (RESET)
   - Arduino pin 11 → ATtiny85 pin 5 (MOSI)
   - Arduino pin 12 → ATtiny85 pin 6 (MISO)
   - Arduino pin 13 → ATtiny85 pin 7 (SCK)
   - Arduino GND → ATtiny85 pin 4 (GND)
   - Arduino 5V → ATtiny85 pin 8 (VCC)

2. Upload ArduinoISP sketch:
   - Open File → Examples → ArduinoISP
   - Upload to Arduino Uno

### Step 3: Minimal Connections

#### Power Supply
1. Connect 5V regulated power to ATtiny85 pin 8 (VCC)
2. Connect ground to ATtiny85 pin 4 (GND)
3. Add 10µF decoupling capacitor between VCC and GND (recommended)

#### BLDC Hall Sensor
1. **Option 1 - Single Hall Sensor (Standard setup):**
   - Connect ANY Hall wire (A, B, or C) to ATtiny85 physical pin 2 (PB3)
   - Provides 2 pulses per electrical revolution
2. **Option 2 - Composite Hall Sensor (Enhanced performance):**
   - Use external OR gate IC (e.g., 74HC32) or diode OR circuit
   - Combine all three Hall sensor outputs
   - Connect composite signal to ATtiny85 physical pin 2 (PB3)
   - Provides 6 pulses per electrical revolution
3. Connect motor Hall sensor power to ATtiny85 VCC (pin 8)
4. Connect motor Hall sensor ground to ATtiny85 GND (pin 4)

#### PWM Output
1. Connect ATtiny85 physical pin 5 (PB0) to ESC signal input

### Step 4: Programming ATtiny85

1. Select board in Arduino IDE:
   - Tools → Board → ATtiny25/45/85
   - Tools → Processor → ATtiny85
   - Tools → Clock → 8 MHz (internal)
   - Tools → Programmer → Arduino as ISP

2. Upload code to ATtiny85

### Step 5: Final Assembly

1. Disconnect programming connections
2. Connect motor to ESC
3. Apply power to system

## Hall Sensor Signal Composition

### Understanding Hall Sensor Signals

BLDC motors with 3 Hall sensors provide position feedback through three digital signals. Each Hall sensor outputs a square wave as the motor rotates:

- **Single Hall sensor**: Provides 2 pulses per electrical revolution (120° electrical)
- **All three Hall sensors combined**: Provides 6 pulses per electrical revolution (60° electrical)

### Benefits of Composite Signals

- **Higher Resolution**: More frequent position updates
- **Smoother Control**: Better motor speed regulation
- **Redundant Information**: Three sensors provide backup if one fails
- **Improved Reliability**: Less sensitive to individual sensor variations

### Implementation Methods

#### Method 1: Diode OR Circuit (Simple, No IC Required)

**Components needed:**
- 3x small signal diodes (1N4148 or similar)
- 1x 4.7kΩ resistor

**Circuit:**
```
Hall A ──>|───┐
Hall B ──>|───┼─── 4.7kΩ ──── Microcontroller Input Pin
Hall C ──>|───┘
              │
              GND
```

**How it works:**
- Each diode allows current to flow only when its Hall sensor is HIGH
- The OR logic combines all three signals
- Pull-down resistor ensures LOW state when all sensors are LOW

#### Method 2: OR Gate IC (Professional)

**Components needed:**
- 1x OR gate IC (74HC32 or similar)
- 1x 4.7kΩ resistor (optional pull-down)

**Circuit:**
```
Hall A ──┬─── OR Gate ──── 4.7kΩ ──── Microcontroller Input Pin
Hall B ──┼─── Input
Hall C ──┘
```

**How it works:**
- Dedicated OR gate combines all three inputs
- Clean digital signal output
- No diodes needed

### Code Considerations

When using composite signals, update the pulse configuration:

```cpp
// For single Hall sensor (2 pulses per electrical revolution)
#define PULSES_PER_REV 2

// For composite signal (6 pulses per electrical revolution)
// #define PULSES_PER_REV 6
```

### Testing Signal Composition

1. **Verify individual sensors:**
   - Connect each Hall wire individually to oscilloscope
   - Confirm each provides clean square wave during motor rotation

2. **Test composite circuit:**
   - Connect composite output to oscilloscope
   - Verify combined signal provides expected pulse frequency
   - Check for clean transitions and proper voltage levels

3. **Motor testing:**
   - Run motor at low speed first
   - Monitor RPM stability and smoothness
   - Compare performance with single sensor vs composite signal

## Software Installation

### Arduino Uno Version

1. Download project files from repository
2. Open Arduino IDE
3. Navigate to `arduino_uno/arduino_uno.ino`
4. Select Tools → Board → Arduino Uno
5. Select correct COM port
6. Click Upload button

### ATtiny85 Version

1. Download project files from repository
2. Open Arduino IDE
3. Navigate to `attiny85/attiny85.ino`
4. Configure board settings:
   - Tools → Board → ATtiny25/45/85
   - Tools → Processor → ATtiny85
   - Tools → Clock → 8 MHz (internal)
   - Tools → Programmer → Arduino as ISP
5. Click Upload button

### Configuration Verification

#### Arduino Uno
1. Open Serial Monitor (115200 baud)
2. Verify startup message appears
3. Check that "BLDC PID Controller Started" message displays

#### ATtiny85
1. System runs autonomously after power-up
2. No serial output available for verification
3. Motor operation indicates successful programming

## Initial Testing

### Safety Precautions
1. Remove propeller or load from motor
2. Ensure adequate ventilation
3. Have power disconnect available
4. Test at low power levels first

### Arduino Uno Basic Test

1. Power on system without motor connected to ESC
2. Open Serial Monitor (115200 baud)
3. Verify startup messages
4. Send `GET PARAMS` command
5. Verify parameter display

### Motor Connection Test

1. Connect motor to ESC
2. Set mode switch to production mode (open/floating)
3. Apply power
4. Monitor motor startup behavior
5. Motor should ramp up smoothly to DEFAULT_TARGET_RPM

### ESC Calibration (if required)

Some ESCs require calibration:
1. Disconnect motor from ESC
2. Power on ESC
3. Send maximum PWM (255) for 2 seconds
4. Send minimum PWM (0) for 2 seconds
5. ESC should beep to confirm calibration

## PID Tuning Procedure

### Arduino Uno Potentiometer Tuning

1. Set mode switch to tuning position (closed to GND)
2. Connect all four potentiometers (A0-A3)
3. Power on system
4. Open Serial Plotter (Tools → Serial Plotter)
5. Adjust potentiometers:
   - **PPR (A0)**: Pulses per revolution (typically 6 for 3-Hall motors)
   - **Kp**: Start low, increase until oscillation begins, then reduce slightly
   - **Ki**: Start at 0, increase slowly to eliminate steady-state error
   - **Kd**: Start at 0, increase to reduce overshoot

### Arduino Uno Serial Tuning

1. Set mode switch to production position
2. Open Serial Monitor (115200 baud)
3. Send `MODE SERIAL` command
4. Use SET commands:
   ```
   SET TARGET DEFAULT_TARGET_RPM
   SET KP DEFAULT_KP
   SET KI DEFAULT_KI
   SET KD DEFAULT_KD
   ```
5. Send `SAVE` to store parameters in EEPROM
6. Monitor response in Serial Plotter

### Parameter Optimization

1. **Proportional Gain (Kp)**: Controls response speed
   - Too low: Slow response, steady-state error
   - Too high: Oscillation, instability

2. **Integral Gain (Ki)**: Eliminates steady-state error
   - Too low: Persistent error
   - Too high: Windup, oscillation

3. **Derivative Gain (Kd)**: Reduces overshoot
   - Too low: Overshoot and ringing
   - Too high: Slow response, noise sensitivity

## Production Deployment

### Arduino Uno Production Setup

1. Tune PID parameters using potentiometer tuning mode
2. Note the optimal potentiometer positions for PPR, Kp, Ki, Kd
3. Set mode switch to production position (open/floating)
4. Update `PRODUCTION_KP`, `PRODUCTION_KI`, `PRODUCTION_KD` constants in `config.h` if needed
5. Re-upload code with tuned parameters
6. Secure all wiring connections
7. Install in final enclosure

### ATtiny85 Production Setup

1. Tune parameters using Arduino Uno version
2. Update `PRODUCTION_*` constants in `attiny85/config.h`
3. Upload tuned code to ATtiny85
4. Connect only required wires:
   - Hall sensor to pin 2 (PB3)
   - PWM output to pin 5 (PB0)
   - Power and ground
5. Seal connections and install in enclosure

## Troubleshooting

### Motor Not Starting

**Symptoms**: Motor does not spin, no response to commands

**Possible Causes and Solutions**:
1. **Power Supply Issues**:
   - Verify ESC power connections
   - Check motor power supply voltage and current capacity
   - Ensure common ground between Arduino and ESC

2. **ESC Configuration**:
   - Verify ESC supports PWM input (not servo pulses)
   - Check ESC calibration procedure
   - Confirm PWM frequency compatibility

3. **Signal Connections**:
   - Verify PWM output pin connection (Arduino pin 9, ATtiny85 pin 5)
   - Check for loose or damaged wires
   - Use oscilloscope to verify PWM signal presence

### Incorrect RPM Reading

**Symptoms**: Motor speed does not match target, erratic behavior

**Possible Causes and Solutions**:
1. **Hall Sensor Issues**:
   - Verify Hall sensor connection (Arduino pin 2, ATtiny85 pin 2)
   - Check Hall sensor power supply (5V)
   - Confirm motor has 3-Hall sensors (not single Hall)

2. **Pulse Configuration**:
   - Verify `PULSES_PER_REV` is set to 6 for 3-Hall BLDC motors
   - Check that any Hall wire (A, B, or C) is connected

3. **Noise Interference**:
   - Use shielded cable for Hall sensor connections
   - Keep Hall sensor wires away from power cables
   - Add pull-up resistor if signal is weak

### Unstable Control

**Symptoms**: Motor oscillates, hunts, or runs away

**Possible Causes and Solutions**:
1. **PID Parameters**:
   - Reduce Kp if oscillating
   - Reduce Ki if windup occurs
   - Adjust Kd for overshoot control

2. **Load Variations**:
   - Account for load changes in PID tuning
   - Consider feed-forward control for known loads

3. **Timing Issues**:
   - Verify control loop runs at 50Hz
   - Check for timing interference from other operations

### Programming Issues

**Symptoms**: Upload fails, board not recognized

**Solutions**:
1. **Arduino Uno**:
   - Verify correct board selection in Tools menu
   - Check COM port selection
   - Try different USB cable or port

2. **ATtiny85**:
   - Verify ArduinoISP connections
   - Check ATtiny85 fuse settings
   - Confirm 10µF capacitor on RESET pin during programming

### Power Supply Issues

**Symptoms**: Motor behavior erratic or unstable

**Solutions**:
1. **Arduino Uno**: Ensure stable 5V supply
2. **ATtiny85**: Verify clean power input
3. **Hardware**: Check power supply filtering

## Safety Considerations

### Electrical Safety
1. **Power Supply Isolation**: Keep motor power supply separate from control electronics
2. **Grounding**: Maintain common ground between all system components
3. **Voltage Compatibility**: Verify all components operate within specified voltage ranges
4. **Current Protection**: Use appropriately rated power supplies and wiring

### Mechanical Safety
1. **Load Removal**: Remove propellers or loads during initial testing
2. **Ventilation**: Ensure adequate cooling for motor and electronics
3. **Mounting**: Secure all components to prevent vibration damage
4. **Emergency Access**: Provide easy access to power disconnect

### Operational Safety
1. **Gradual Testing**: Start at low power levels and gradually increase
2. **Monitoring**: Continuously monitor system behavior during operation
3. **Parameter Limits**: Implement software limits on PID parameters
4. **Fail-Safe Operation**: Design for safe failure modes

### Watchdog Timer
- **Purpose**: Automatic recovery from software hangs
- **Timeout**: 4 seconds (Arduino Uno), 8 seconds (ATtiny85)
- **Recovery**: Graceful system operation

### Safety Features
- **Anti-windup Protection**: Prevents integrator runaway
- **Output Limiting**: Constrains PWM to safe range
- **Debounce Filtering**: Rejects electrical noise

Follow all local electrical and safety codes. If unsure about any aspect of the installation, consult a qualified electrical engineer.
