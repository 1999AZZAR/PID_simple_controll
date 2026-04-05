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
|  [Trim enable] → 3  |  // LOW = apply sensitivity pot, HIGH = scale 1.0
|                     |
|  [PWM to ESC] ← 9   |  // PWM output
|                     |
|  [Sensitivity pot] → A4 |  // scales Kp/Ki/Kd together
+---------------------+
```

## Connections

### BLDC Hall Sensor
- Any Hall wire (A, B, or C) → Arduino Pin 2
- Hall sensors share power/ground with Arduino (5V/GND)
- No additional components needed for most BLDC motors

### Trim enable (optional)
- One side → Arduino Pin 3
- Other side → GND (when closed = sensitivity pot active)

### PWM Output to ESC
- Arduino Pin 9 → ESC signal input
- ESC power and motor connections as per ESC manual

### Sensitivity pot (optional, 10kΩ linear or trimmer)
- Wiper → A4, ends → GND and 5V

## Power Supply
- **Arduino**: USB or external 7-12V
- **Motor/ESC**: Separate supply as required by motor
- **Common ground** between Arduino and motor supply

## BLDC Hall Sensor Configuration

### 8-Pole BLDC Motor Compatibility
- Designed for 8-pole BLDC motors with built-in Hall sensors
- Compatible with any 3-Hall BLDC motor (Hall A, Hall B, Hall C)
- Single Hall sensor provides 4 pulses per mechanical revolution for 8-pole motors
- Uses period measurement for accurate RPM calculation at low speeds
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
- System configured for 8-pole BLDC motors with single Hall sensor
- Fixed at 4 pulses per mechanical revolution for optimal performance
- Uses period measurement instead of pulse counting for better low-speed stability
- No calibration required - PPR is fixed in configuration
