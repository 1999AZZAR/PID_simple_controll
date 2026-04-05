# Arduino Uno BLDC PID Controller

**Simplified PID controller** for BLDC motors with optional PID sensitivity trim and serial monitoring.

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

The Arduino Uno version keeps a fixed target RPM (`DEFAULT_TARGET_RPM`, 1440) and optionally scales Kp, Ki, and Kd together with one trim pot for field calibration.

### Key Features

- **Fixed target RPM**: Always `DEFAULT_TARGET_RPM` from `config_common.h`
- **Sensitivity trim**: Pull D3 LOW and use A4 to scale all three gains (about 0.75x to 1.25x)
- **Serial Plotter**: Real-time visualization including gain scale
- **Modular configuration**: Pins and limits in `config.h`
- **Simplified design**: No serial commands or EEPROM

## Quick Start

1. **Hardware Setup**: Connect components according to pin assignments below
2. **Upload Code**: Load `arduino_uno.ino` to Arduino Uno
3. **Configure**: Modify `config.h` for your specific setup
4. **Optional trim**: Use D3 + A4 to fine-tune PID sensitivity on the bench or PCB
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

### Optional components (sensitivity trim)
- 1x 10kΩ potentiometer (or PCB trimmer) on A4
- Jumper or switch from D3 to GND to enable trim

### Pin Connections

| Component | Arduino Pin | Description |
|-----------|-------------|-------------|
| BLDC Hall Sensor | Digital Pin 2 | Any Hall wire from motor (interrupt pin) - provides 4 pulses per mechanical revolution |
| PWM Output | Digital Pin 9 | PWM signal to ESC |
| Trim enable | Digital Pin 3 | Pull LOW to apply sensitivity pot (internal pullup). HIGH = scale 1.0 |
| Sensitivity pot | Analog A4 | Wiper scales Kp, Ki, Kd together; target RPM stays 1440 |

### Hall Sensor Signal Options

The controller uses a single Hall sensor wire for RPM feedback, but for enhanced performance, you can implement a composite signal from all three Hall sensors:

#### Single Hall Sensor (Current Implementation)
- Connect any one Hall wire (A, B, or C) to Arduino Pin 2
- Provides 4 pulses per mechanical revolution for 8-pole motors
- Uses period measurement for accurate RPM calculation at low speeds
- Simple wiring, reliable operation

#### Composite Hall Sensor Signal (Advanced)
- Combine all three Hall sensors using OR logic
- Provides 12 pulses per mechanical revolution for 8-pole motors
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

### Pin definitions
```cpp
#define RPM_SENSOR_PIN        2   // Hall sensor input (interrupt)
#define PWM_OUTPUT_PIN        9   // PWM output to ESC
#define POT_ENABLE_PIN        3   // Pull LOW to enable sensitivity trim
#define POT_SENSITIVITY_PIN   A4  // Trims Kp/Ki/Kd scale
#define PID_SENSITIVITY_MIN   0.75f
#define PID_SENSITIVITY_MAX   1.25f
```

### Control Parameters
```cpp
#define DEFAULT_TARGET_RPM     1440.0  // Default target RPM
#define DEFAULT_KP             3.25    // Default proportional gain
#define DEFAULT_KI             0.0320  // Default integral gain
#define DEFAULT_KD             0.001   // Default derivative gain
#define DEFAULT_PULSES_PER_REV 24      // Default pulses per revolution
```

### Safety Settings
```cpp
#define SOFT_START_DURATION_MS  2000  // Soft-start ramp time
#define MIN_PULSE_WIDTH_US      100   // Debounce filter threshold

// Safety features (recommended to keep enabled)
#define WATCHDOG_ENABLED           true   // Hardware watchdog protection (4s timeout)
// Emergency stop feature removed for simplified operation
```

## Operating modes

Digital pin 3 selects whether the sensitivity pot on A4 is active.

### Production (default)
- **Pin 3**: HIGH or floating
- **Behavior**: Kp, Ki, Kd equal `DEFAULT_KP`, `DEFAULT_KI`, `DEFAULT_KD`; target RPM is `DEFAULT_TARGET_RPM`

### Sensitivity trim
- **Pin 3**: LOW
- **Behavior**: Same target RPM; pot on A4 sets a multiplier on all three gains (`PID_SENSITIVITY_MIN` to `PID_SENSITIVITY_MAX`)
- **Use case**: Calibrate each unit or ESC so speed settles cleanly on 1440 RPM without changing firmware

## Serial Plotter Output

The system outputs comma-separated values:
```
Target,Current,Error,PID_Output,PWM,Kp,Ki,Kd,PPR,Sens
```

- **Sens**: Current gain scale (1.0 when trim disabled)

### Plotter setup
1. Open Arduino IDE, Serial Plotter, 115200 baud
2. Traces include live gains and sensitivity multiplier

## Tuning procedure

### Sensitivity trim on hardware
1. Flash firmware with baseline `DEFAULT_KP`, `DEFAULT_KI`, `DEFAULT_KD` in `config_common.h`
2. Tie D3 LOW (trim on), connect trim pot to A4
3. Run at load and adjust the pot until speed is stable on 1440 RPM with acceptable overshoot
4. For sealed production, leave D3 HIGH or strap per your PCB after setting the trimmer once in test

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
- **Control Frequency**: 50Hz main loop, 20Hz RPM calculation
- **PWM Frequency**: ~490Hz (Timer0 with prescaler 8)
- **Interrupt Response**: Hall sensor debounce filter prevents false triggers
- **RPM Calculation**: Period measurement provides stable low-speed operation
- **Simplified Design**: No EEPROM wear or serial command overhead
