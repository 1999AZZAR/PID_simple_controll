# BLDC Motor PID Controller

A robust PID control system for maintaining a BLDC motor at exactly 1440 RPM, with implementations for both development (Arduino Uno) and production (ATTiny85) deployment.

## Quick Start

Choose your implementation based on your needs:

### For Development & Tuning
Use the **Arduino Uno version** (`arduino_uno/`) which includes:
- Serial command interface for real-time PID tuning
- EEPROM parameter storage
- Serial Plotter visualization
- Full debugging capabilities

### For Production Deployment
Use the **ATTiny85 version** (`attiny85/`) which provides:
- Minimal resource usage (31% flash, 8% RAM)
- Optimized for power efficiency
- Pre-tuned PID parameters
- Cost-effective production solution

## Project Structure

```
BLDC_PID_Controller/
├── arduino_uno/                 # Development implementation
│   ├── arduino_uno.ino          # Arduino Uno code with serial tuning
│   ├── hardware_schematic.txt   # Hardware setup guide
│   └── README.md               # Detailed Arduino Uno documentation
├── attiny85/                    # Production implementation
│   ├── attiny85.ino            # Production ATTiny85 code
│   ├── ATTiny85_README.md       # ATTiny85 documentation
│   └── ATTiny85_hardware_schematic.txt # ATTiny85 hardware setup
└── README.md                   # This overview file
```

## Getting Started

1. **Choose your platform:**
   - Open `arduino_uno/arduino_uno.ino` for development
   - Open `attiny85/attiny85.ino` for production

2. **Follow the platform-specific README:**
   - `arduino_uno/README.md` for Arduino Uno setup
   - `attiny85/ATTiny85_README.md` for ATTiny85 setup

3. **Hardware setup:** Refer to the schematic files in each platform folder

## Key Features

- **PID Control**: Precise motor speed regulation with anti-windup protection
- **Multiple Tuning Methods**: Potentiometers, serial commands, or pre-tuned parameters
- **Real-time Monitoring**: RPM feedback with configurable update rates
- **Parameter Persistence**: EEPROM storage for tuned parameters (Arduino Uno)
- **Cross-Platform**: Development and production implementations

## Requirements

- BLDC motor with ESC
- RPM sensor (Hall effect or optical)
- Power supply suitable for your motor
- Arduino Uno (for development) or ATTiny85 (for production)

## License

This project is released under the MIT License.
