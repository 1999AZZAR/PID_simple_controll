# PWM_TEST - PWM Cycling Test Programs

This folder contains PWM (Pulse Width Modulation) test programs for both ATtiny microcontrollers and Arduino boards. These programs cycle PWM output values from 0 to 255 and back down continuously, making them ideal for testing PWM functionality, motor control, LED brightness control, or any PWM-driven device.

## Folder Structure

```
PWM_TEST/
├── README.md          # This file
├── attiny/
│   └── attiny.ino     # ATtiny PWM test program
└── arduino/
    └── arduino.ino    # Arduino PWM test program
```

## Programs Overview

### ATtiny Version (`attiny/attiny.ino`)

**Target Hardware:** ATtiny25/45/85 microcontrollers
**PWM Pin:** Arduino pin 0 (PB0, physical pin 5)
**PWM Frequency:** ~30.64 Hz (configured via Timer0)
**Purpose:** Low-power PWM testing for ATtiny microcontrollers

**Features:**
- Cycles PWM values from 0 to 255 and back down continuously
- 10ms delay between steps for smooth transitions
- Optimized for ATtiny's limited resources
- Uses Arduino-style pin numbering for consistency

**Hardware Setup:**
```
ATtiny85 Pinout:
Physical Pin 1: VCC (5V)
Physical Pin 2: PB3 (Arduino pin 3)
Physical Pin 3: PB4 (Arduino pin 4)
Physical Pin 4: GND
Physical Pin 5: PB0 (Arduino pin 0) ← PWM OUTPUT
Physical Pin 6: PB1 (Arduino pin 1)
Physical Pin 7: PB2 (Arduino pin 2)
Physical Pin 8: Reset
```

**Programming:**
- Requires Arduino IDE with ATtiny support (via ATTinyCore or similar)
- Select appropriate ATtiny board and 8MHz internal clock
- Upload via ISP programmer

### Arduino Version (`arduino/arduino.ino`)

**Target Hardware:** Arduino Uno, Nano, Mega, or compatible boards
**PWM Pin:** Arduino pin 9 (OC1A, Timer1)
**PWM Frequency:** ~490 Hz (Arduino default)
**Purpose:** Standard PWM testing for Arduino boards

**Features:**
- Same PWM cycling behavior as ATtiny version
- Uses Arduino's standard PWM capabilities
- Pin 9 provides clean PWM signal via Timer1
- Easy to modify for different PWM pins

**Hardware Setup:**
```
Arduino Uno Pinout:
Digital Pin 9 ← PWM OUTPUT (OC1A)
GND - Ground connection
5V - Power connection
```

**Programming:**
- Standard Arduino IDE setup
- Select appropriate Arduino board
- Direct USB upload

## Usage Instructions

### Basic Testing

1. **Connect PWM output** to your test device (LED with resistor, motor controller, etc.)
2. **Upload the appropriate program** for your microcontroller
3. **Power the circuit** and observe the PWM cycling effect
4. **Verify smooth transitions** from minimum to maximum PWM values

### Testing with Motors

For BLDC motor testing:
- Connect PWM output to ESC (Electronic Speed Controller) signal input
- Ensure proper power supply for motor and ESC
- Monitor motor speed changes as PWM cycles

### Testing with LEDs

For LED brightness testing:
- Connect LED with appropriate resistor (220Ω-1KΩ) to PWM pin
- LED should smoothly fade from off to full brightness and back

## Code Customization

### Changing PWM Speed
Modify the `delayMs` variable:
```cpp
int delayMs = 10;  // Faster cycling
int delayMs = 50;  // Slower cycling
```

### Changing PWM Range
Modify the boundary conditions:
```cpp
if (pwmValue >= 255)  // Change maximum value
if (pwmValue <= 0)    // Change minimum value
```

### Different PWM Pins (Arduino only)
Change the PWM pin (must be PWM-capable pin):
```cpp
int PWM_Pin = 9;  // Pin 9 (Timer1)
int PWM_Pin = 3;  // Pin 3 (Timer2)
int PWM_Pin = 5;  // Pin 5 (Timer0)
int PWM_Pin = 6;  // Pin 6 (Timer0)
```

## Technical Details

### PWM Resolution
- Both versions use 8-bit PWM (0-255 range)
- ATtiny version: Manual Timer0 configuration for specific frequency
- Arduino version: Hardware PWM with Arduino defaults

### Timing
- PWM cycling period: ~5.12 seconds (255 × 2 × 10ms)
- Adjustable via `delayMs` variable
- Non-blocking operation (no delays in PWM updates)

## Troubleshooting

### ATtiny Issues
- **No PWM output:** Check fuse settings and clock configuration
- **Wrong frequency:** Verify Timer0 configuration matches your ATtiny variant
- **Upload fails:** Ensure proper ISP programmer connection

### Arduino Issues
- **No PWM output:** Verify pin selection (must be PWM-capable)
- **Wrong pin:** Check Arduino pinout for PWM-capable pins
- **Timing issues:** Reduce delay if cycling too slowly

## Related Files

- `../ATTiny40PMW261125.ino` - Original PWM test reference
- `../../attiny85/` - Full PID controller implementation for ATtiny85
- `../../arduino_uno/` - Full PID controller implementation for Arduino

## Author

**azzar budiyanto**
- Date: November 2025
- Part of PID_simple_controll project

## License

This code is part of the PID_simple_controll project. See main project README for licensing information.
