# BLDC PID Controller - ATtiny85 Production Version

## Table of Contents

- [Production Configuration](#production-configuration)
- [ATtiny85 Physical Pin Mapping](#attiny85-physical-pin-mapping)
- [Required Connections (Only 2 Pins!)](#required-connections-only-2-pins)
- [Programming Connections](#programming-connections)
- [Power Supply](#power-supply)
- [Limitations & Workarounds](#limitations--workarounds)
- [Tuning Procedure (ATtiny85)](#tuning-procedure-attiny85)
- [Alternative Tuning Method](#alternative-tuning-method)
- [Compilation Notes](#compilation-notes)
- [Fuse Settings](#fuse-settings)
- [Troubleshooting](#troubleshooting)

## Production Configuration
**No Potentiometers Required!** All PID gains are pre-tuned and hardcoded from Arduino development.

## ATtiny85 Physical Pin Mapping

```
+---------------------+
|  RST (1)  VCC (8)  |
|  PB3 (2)  PB2 (7)  |
|  PB4 (3)  PB1 (6)  |
|  GND (4)  PB0 (5)  |
+---------------------+
```

## Required Connections (Only 2 Pins!)

### BLDC Hall Sensor Input
- Any Hall wire (A, B, or C) from 3-Hall BLDC motor → ATtiny85 Physical Pin 2 (PB3)
- Compatible with motors like 42BLF20-22.0223 and similar 3-Hall BLDC motors
- Hall sensors share power/ground with ATtiny85 (5V/GND)
- **Note**: PB3 is interrupt-capable (INT0) for accurate period measurement
- 4 pulses per mechanical revolution from 8-pole BLDC motors with single Hall sensor

### PWM Output to ESC
- ATtiny85 Physical Pin 5 (PB0) → ESC signal input
- **Note**: Timer0-based PWM (~1kHz frequency)

## Programming Connections

When uploading code:
- Use Arduino as ISP or dedicated programmer
- Connect RESET, MOSI, MISO, SCK pins
- Remove motor power during programming

## Power Supply

- **ATtiny85**: 2.7-5.5V (use 5V for compatibility)
- **Motor/ESC**: Separate supply with common ground
- Add decoupling capacitor (10µF) near VCC/GND pins

## Limitations & Workarounds

### 1. Ultra-Minimal Design
- **Zero Controls**: No potentiometers, switches, or user adjustments
- **Minimal Safety**: Watchdog timer only
- **2 Pins Only**: Hall sensor input + PWM output (pure production)
- **Pre-tuned Values**: All PID parameters hardcoded from Arduino development

### 2. PWM Limitations
- Only one PWM output pin available
- Timer0 shared between PWM and system timing
- ~1kHz PWM frequency (may need adjustment for ESC)

### 3. No ADC Usage
- No analog inputs needed (production version)
- All values hardcoded for reliability
- No potentiometer calibration required

### 4. Memory Constraints
- 512 bytes SRAM (careful with variables)
- No Serial output for debugging

## Deployment Procedure

**ATTiny85 has NO controls or safety features - it's ultra-minimal production hardware!**

1. **Tune on Arduino First**: Use Arduino Uno with potentiometers to find optimal PID values
2. **Update Constants**: Copy tuned values to `PRODUCTION_*` constants in `attiny85/config.h`
3. **Program ATTiny85**: Upload code with Arduino as ISP programmer
4. **Minimal Connections**: Connect only Hall sensor and PWM output (2 pins total)
5. **Power On**: System runs autonomously at pre-tuned 1440 RPM

## Why Ultra-Minimal?

- **Cost Optimization**: $2 microcontroller vs $20 Arduino
- **Size Efficiency**: 77% of 2KB flash (1,586 bytes used)
- **Pin Minimization**: Only 2 pins required for operation
- **Zero Maintenance**: No calibration or adjustment needed
- **Production Ready**: Deploy and forget design

**Result**: Ultra-cheap, ultra-simple 2-wire motor control! 💰🔧

## Compilation Notes

- Use ATtiny85 board definition in Arduino IDE
- Clock: 8MHz internal
- Programmer: Arduino as ISP
- Optimize code for size (-Os)
- Remove unused functions

## Fuse Settings

- **Low**: 0xE2 (8MHz internal clock)
- **High**: 0xDF (SPM disabled)
- **Extended**: 0xFF (default)

## Troubleshooting

### 1. Motor Not Responding
- Check PWM frequency compatibility with ESC
- Verify PWM pin connection (physical pin 5)
- Test with fixed PWM value first

### 2. RPM Not Reading
- Verify BLDC Hall sensor connection to physical pin 2 (PB3)
- Ensure motor is a 3-Hall BLDC type (like 42BLF20-22.0223)
- Check that you're using any of the three Hall wires (Hall A, B, or C)
- Verify Hall sensors share 5V/GND with ATtiny85
- Confirm `PULSES_PER_REV` is set to 6 for 3-Hall BLDC motors

### 3. Unstable Control
- Reduce PID gains (start with Arduino-tuned values)
- Check timing calculations
- Verify control loop frequency

### 4. Programming Issues
- Ensure proper ISP connections
- Check fuse settings
- Try external crystal if 8MHz internal unstable
