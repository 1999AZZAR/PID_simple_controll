# BLDC PID Controller - Arduino Uno Hardware Schematic

## Table of Contents

- [Arduino Board Layout](#arduino-board-layout)
- [Connections](#connections)
- [Power Supply](#power-supply)
- [BLDC Hall Sensor Configuration](#bldc-hall-sensor-configuration)
- [Wiring Notes](#wiring-notes)
- [Calibration](#calibration)

## Arduino Board Layout

```
+---------------------+
|                     |
|  [RPM Sensor] → 2   |  // Interrupt pin
|                     |
|  [Mode Switch] → 3  |  // LOW = Tuning, HIGH = Production
|                     |
|  [PWM to ESC] ← 9   |  // PWM output
|                     |
|  [Target RPM Pot] → A0 |
|  [Kp Pot] → A1      |
|  [Ki Pot] → A2      |
|  [Kd Pot] → A3      |
+---------------------+
```

## Connections

### BLDC Hall Sensor
- Any Hall wire (A, B, or C) → Arduino Pin 2
- Hall sensors share power/ground with Arduino (5V/GND)
- No additional components needed for most BLDC motors

### Mode Switch
- One side → Arduino Pin 3
- Other side → GND (when closed = tuning mode)

### PWM Output to ESC
- Arduino Pin 9 → ESC signal input
- ESC power and motor connections as per ESC manual

### Potentiometers (4x 10kΩ linear)
- **Target RPM**: Wiper → A0, ends → GND and 5V
- **Kp**: Wiper → A1, ends → GND and 5V
- **Ki**: Wiper → A2, ends → GND and 5V
- **Kd**: Wiper → A3, ends → GND and 5V

## Power Supply
- **Arduino**: USB or external 7-12V
- **Motor/ESC**: Separate supply as required by motor
- **Common ground** between Arduino and motor supply

## BLDC Hall Sensor Configuration

### 3-Hall BLDC Motor Compatibility
- Designed for motors like 42BLF20-22.0223 with built-in Hall sensors
- Compatible with any 3-Hall BLDC motor (Hall A, Hall B, Hall C)
- Each Hall sensor provides 2 pulses per electrical revolution
- Total: 6 pulses per electrical revolution when using any single Hall wire
- Connect ANY Hall wire (A, B, or C) to Arduino Pin 2
- Hall sensors operate at 5V, compatible with Arduino Uno
- No diode isolation required between controller and motor Hall sensors

## Wiring Notes

1. Use shielded cable for RPM sensor if experiencing noise
2. Keep PWM wires away from sensor wires to prevent interference
3. Use appropriate wire gauge for motor power (ESC requirements)
4. Add pull-up resistor (4.7kΩ) on RPM sensor if signal is weak
5. Ground planes should be connected between Arduino and ESC

## Calibration

### ESC Calibration (if required)
1. Disconnect motor from ESC
2. Power on ESC
3. Send full throttle PWM (255) for 2 seconds
4. Send zero throttle PWM (0) for 2 seconds
5. ESC should beep to confirm calibration

### RPM Sensor Calibration
- `PULSES_PER_REV` is set to 6 for 3-Hall BLDC motors
- If using a different sensor type, adjust this constant:
  - Single Hall sensor: Usually 2 pulses per revolution
  - Optical encoder: Check datasheet for pulses per revolution
  - Tachometer: Usually 1 pulse per revolution
  - Other sensors: Measure actual pulses per revolution
