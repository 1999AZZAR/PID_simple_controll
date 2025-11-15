# BLDC PID Controller - ATtiny85 Production Version

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
- **Note**: PB3 is interrupt-capable (INT0) for accurate RPM counting
- 6 pulses per electrical revolution from 3-Hall BLDC motors

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

### 1. Reduced Potentiometers
- Only 3 potentiometers instead of 4
- Ki and Kd share one potentiometer (time-multiplexed)

### 2. PWM Limitations
- Only one PWM output pin available
- Timer0 shared between PWM and system timing
- ~1kHz PWM frequency (may need adjustment for ESC)

### 3. ADC Constraints
- ADC2 (PB4) used for Kp potentiometer
- ADC3 (PB2) shared between mode switch and Ki/Kd pot
- ADC1 (PB1) used for target RPM pot

### 4. Memory Constraints
- 512 bytes SRAM (careful with variables)
- No Serial output for debugging

## Tuning Procedure (ATtiny85)

1. Upload code with Arduino as ISP
2. Connect potentiometers as shown (if using Arduino version for tuning)
3. Connect BLDC Hall sensor to physical pin 2
4. Set mode switch to tuning position (GND)
5. Power on system
6. Adjust potentiometers:
   - Target RPM: Set to 1440
   - Kp: Start low (0.1), increase until oscillation
   - Ki/Kd: Wait for parameter to appear, adjust accordingly
7. Note optimal values
8. Update `PRODUCTION_*` constants in code
9. Re-upload production version with tuned values
10. Connect only Hall sensor and PWM output for final deployment

## Alternative Tuning Method

Since Serial is not available, consider:
- Using LED blink patterns to indicate current values
- EEPROM storage of tuned parameters
- Fixed tuning values based on Arduino testing

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
