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

- **Fixed Target Speed**: `DEFAULT_TARGET_RPM` constant speed control
- **Potentiometer Tuning**: 4 potentiometers for real-time parameter adjustment
- **Serial Plotter Integration**: Real-time visualization of control performance
- **Modular Configuration**: All settings centralized in `config.h`
- **Two Operating Modes**: Production (fixed parameters) and Potentiometer Tuning
- **Simplified Design**: No serial commands or EEPROM storage for maximum reliability
- **Enhanced PID with Spike Dampening**: Derivative-on-measurement, derivative filtering, output slew rate limiting
- **Setpoint Ramping**: Smooth startup transitions to prevent initial overshoot

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
| BLDC Hall Sensor | Digital Pin 2 | Any Hall wire from motor (interrupt pin) - provides 4 pulses per mechanical revolution |
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
#define DEFAULT_TARGET_RPM     1440.0  // Default target RPM
#define DEFAULT_KP             0.150   // Default proportional gain
#define DEFAULT_KI             0.080   // Default integral gain
#define DEFAULT_KD             0.015   // Default derivative gain
#define DEFAULT_PULSES_PER_REV 4       // Default pulses per revolution (8-pole motor)
```

### Spike Dampening Parameters
```cpp
#define DERIVATIVE_FILTER_ALPHA 0.2    // Derivative low-pass filter (0.1=smooth, 0.5=responsive)
#define OUTPUT_SLEW_RATE        80.0   // Max PID output change per iteration
#define SETPOINT_RAMP_RATE      50.0   // RPM per iteration for startup ramping
#define SETPOINT_RAMP_ENABLED   true   // Enable/disable setpoint ramping
#define SETPOINT_RAMP_THRESHOLD 100.0  // Only ramp when error exceeds this value
```

### Safety Settings
```cpp
#define SOFT_START_DURATION_MS  2000  // Soft-start ramp time
#define MIN_PULSE_WIDTH_US      100   // Debounce filter threshold
```

### Emergency Handler Settings (Safe Runaway Protection)
```cpp
#define EMERGENCY_ERROR_THRESHOLD  2000.0  // RPM error to trigger emergency
#define EMERGENCY_RAMPDOWN_RATE    5       // PWM reduction per iteration (gradual)
#define EMERGENCY_MIN_PWM          30      // Minimum PWM during emergency (maintains control)
#define EMERGENCY_RECOVERY_TIME_MS 3000    // Time before recovery attempt (ms)
#define EMERGENCY_FULL_STOP        false   // true = ramp to 0, false = ramp to MIN_PWM
```

The emergency handler uses **gradual ramp-down** instead of instant power cutoff to prevent mechanical stress.

## Operating Modes

The Arduino Uno version supports two operating modes selected by the mode switch (Digital Pin 3):

### Production Mode (Default)
- **Mode Switch**: HIGH or floating
- **Behavior**: Uses fixed target RPM (`DEFAULT_TARGET_RPM`) with potentiometer-adjustable PID parameters
- **Features**: Stable, predictable operation
- **Use Case**: Final production deployment

### Potentiometer Tuning Mode
- **Mode Switch**: LOW (connected to GND)
- **Behavior**: Real-time parameter adjustment via 4 potentiometers
- **Monitoring**: Serial Plotter shows live control response
- **Use Case**: Initial PID tuning and testing

## Enhanced PID Algorithm

The controller uses an enhanced PID algorithm with multiple spike dampening techniques:

### Derivative-on-Measurement

Standard PID uses derivative of error: `Kd * (error - previousError)`. This causes "derivative kick" when the setpoint changes suddenly. The enhanced algorithm uses derivative-on-measurement instead:

```cpp
derivative = -Kd * (measurement - previousMeasurement)
```

This eliminates spikes when target RPM changes while still responding to actual motor speed changes.

### Derivative Low-Pass Filter

Raw derivative calculations amplify high-frequency noise. An exponential moving average (EMA) filter smooths the derivative term:

```cpp
filteredDerivative = alpha * rawDerivative + (1 - alpha) * filteredDerivative
```

- `DERIVATIVE_FILTER_ALPHA = 0.2` (default): Smooth response, good noise rejection
- Higher values (0.3-0.5): Faster response, more noise
- Lower values (0.1-0.15): Very smooth, slower response

### Output Slew Rate Limiting

Prevents sudden PWM changes that can cause mechanical stress or electrical spikes:

```cpp
if (outputChange > OUTPUT_SLEW_RATE) {
    output = previousOutput + OUTPUT_SLEW_RATE;
}
```

- `OUTPUT_SLEW_RATE = 80.0` (default): Balanced response
- Higher values: Faster response, more aggressive
- Lower values: Smoother transitions, slower response

### Setpoint Ramping

During startup, the target RPM ramps gradually from 0 to `DEFAULT_TARGET_RPM`:

```cpp
activeSetpoint += SETPOINT_RAMP_RATE;  // Increment each control cycle
```

This prevents large initial errors that cause integral windup and overshoot.

## Safe Emergency Handler

The controller includes a **safe emergency handler** that protects against motor runaway without causing mechanical damage from sudden stops.

### Why Gradual Ramp-Down?

Instant power cutoff (PWM = 0) can cause:
- Mechanical stress on gearboxes, couplings, and belts
- Sudden jerk that damages equipment or film
- Loss of control worse than controlled slowdown
- ESC protection circuits triggering unexpectedly

### Emergency Detection

The handler detects two conditions:

| Condition | Detection | Action |
|-----------|-----------|--------|
| **Overspeed** | `currentRPM > setpoint + threshold` | Gradual ramp-down |
| **Underspeed/Stall** | `currentRPM < setpoint - threshold` | Gradual ramp-down |

### Gradual Ramp-Down Process

```
1. Emergency triggered → Start from current PWM (no jump)
2. Each iteration → Reduce PWM by EMERGENCY_RAMPDOWN_RATE
3. Continue until → PWM reaches EMERGENCY_MIN_PWM (or 0 if FULL_STOP)
4. Wait for → EMERGENCY_RECOVERY_TIME_MS
5. If error recovered → Resume with setpoint ramp from current speed
```

### Recovery Behavior

When conditions return to normal:
- Setpoint ramp restarts from current RPM (smooth re-acceleration)
- Integral term resets to prevent windup
- Hysteresis prevents oscillating in/out of emergency (50% threshold)

### Configuration Options

| Parameter | Default | Description |
|-----------|---------|-------------|
| `EMERGENCY_ERROR_THRESHOLD` | 2000 | RPM error to trigger emergency |
| `EMERGENCY_RAMPDOWN_RATE` | 5 | PWM reduction per cycle |
| `EMERGENCY_MIN_PWM` | 30 | Minimum PWM (maintains some torque) |
| `EMERGENCY_RECOVERY_TIME_MS` | 3000 | Wait time before recovery |
| `EMERGENCY_FULL_STOP` | false | true = stop at 0, false = stop at MIN_PWM |

### Conditional Integral Scaling

When error is large (startup, load changes), integral accumulation is reduced:

```cpp
if (abs(error) > 500.0) {
    integralScale = 500.0 / abs(error);  // Proportionally reduce
}
```

This prevents integral windup during transient conditions.

## Serial Plotter Output

The system outputs comma-separated values for Serial Plotter visualization:
```
Target,Setpoint,Current,Error,PID_Output,PWM,Kp,Ki,Kd,PPR
```

- **Target**: Final desired RPM (1440)
- **Setpoint**: Active ramped setpoint (ramps from 0 to Target during startup)
- **Current**: Measured motor RPM
- **Error**: Difference between Setpoint and Current
- **PID_Output**: Computed PID control value
- **PWM**: Actual PWM value sent to ESC
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

### Initial Spike or Overshoot
- Reduce `SETPOINT_RAMP_RATE` for slower startup (try 25.0)
- Lower `OUTPUT_SLEW_RATE` to limit PWM change rate (try 50.0)
- Decrease `DERIVATIVE_FILTER_ALPHA` for smoother derivative (try 0.1)
- Verify `SETPOINT_RAMP_ENABLED` is `true`

### Oscillations After Startup
- Reduce `Kp` gain (proportional term too aggressive)
- Increase `DERIVATIVE_FILTER_ALPHA` slightly (0.25-0.3)
- Add small amount of `Kd` to dampen oscillations
- Check for mechanical issues (loose coupling, bearing wear)

### Slow Response to Load Changes
- Increase `OUTPUT_SLEW_RATE` (try 100-120)
- Increase `DERIVATIVE_FILTER_ALPHA` (try 0.3)
- Slightly increase `Ki` for faster error correction
- Verify motor/ESC can handle required torque

### Emergency Handler Triggers Too Often
- Increase `EMERGENCY_ERROR_THRESHOLD` (try 2500 or 3000)
- Check for mechanical issues causing speed variations
- Verify RPM sensor is providing clean signal

### Motor Jerks During Emergency
- Decrease `EMERGENCY_RAMPDOWN_RATE` (try 2 or 3)
- Set `EMERGENCY_FULL_STOP` to `false` to maintain minimum control
- Increase `EMERGENCY_MIN_PWM` if motor stalls during emergency

### Recovery Takes Too Long
- Decrease `EMERGENCY_RECOVERY_TIME_MS` (try 2000)
- Check that error threshold hysteresis allows recovery (50% of threshold)

## Performance Notes

- **Memory Usage**: ~18% RAM usage (optimized for reliability)
- **Control Frequency**: 200Hz main loop (5ms period)
- **RPM Calculation**: 100Hz update rate (10ms interval)
- **PWM Frequency**: ~490Hz (Timer1 with prescaler 64)
- **Interrupt Response**: Hall sensor debounce filter prevents false triggers
- **RPM Calculation**: Period measurement provides stable low-speed operation
- **Simplified Design**: No EEPROM wear or serial command overhead

## Spike Dampening Tuning Guide

### Quick Start Values

| Application | DERIVATIVE_FILTER_ALPHA | OUTPUT_SLEW_RATE | SETPOINT_RAMP_RATE |
|-------------|-------------------------|------------------|-------------------|
| Smooth/Quiet | 0.1 | 50 | 25 |
| Balanced (Default) | 0.2 | 80 | 50 |
| Responsive | 0.3 | 120 | 100 |
| Aggressive | 0.5 | 200 | 200 |

### Tuning Procedure

1. **Start with defaults** - Verify system stability
2. **Adjust setpoint ramp** - If overshoot at startup, reduce `SETPOINT_RAMP_RATE`
3. **Tune slew rate** - If oscillating, reduce `OUTPUT_SLEW_RATE`
4. **Fine-tune derivative filter** - If noisy, reduce `DERIVATIVE_FILTER_ALPHA`
5. **Test under load** - Verify performance with actual operating conditions

### Monitoring Tuning Progress

Use Serial Plotter to observe:
- **Setpoint vs Current**: Should track smoothly during ramp-up
- **Error**: Should decrease steadily without large oscillations
- **PID_Output**: Should change gradually, not jump abruptly
