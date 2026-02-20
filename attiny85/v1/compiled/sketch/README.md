#line 1 "/home/azzar/project/microcontrollers/PIDs/PID_simple_controll/attiny85/README.md"
# BLDC PID Controller - ATtiny85 Production Version

**⚠️ FLASHING WARNING ⚠️**
The ATtiny85 is a robust and tiny chip, but **flashing it can be difficult** (requires ISP programmer, capacitor on reset, correct fuses).
For a **easier "small and stable" alternative**, we highly recommend the **[ESP32-C3 Version](../esp32_c3/README.md)** (USB flashing, better performance).

---

This is the production-ready version of the BLDC PID controller running on ATtiny85.
It maintains **exact 1440 RPM** using pre-tuned PID gains from the Arduino Uno development phase.

## New Features (v2.0)
*   **Boosted Kickstart**: Starts motor at PWM 45 (18%) to overcome static friction instantly.
*   **EMA Filter**: Exponential Moving Average (Alpha 0.25) for stable RPM readings.
*   **200Hz Control Loop**: Synchronized with the Arduino Uno version for consistent behavior.

## Hardware Setup
**Minimal Connections (2 Pins Only!)**

| Pin | Function | Connection |
| :--- | :--- | :--- |
| **PB3 (Pin 2)** | RPM Input | Hall Sensor Signal (Any wire A/B/C) |
| **PB0 (Pin 5)** | PWM Output | ESC Signal Input |
| **VCC (Pin 8)** | Power | 5V Supply |
| **GND (Pin 4)** | Ground | Common Ground |

## Configuration
The system uses **Internal 8MHz Oscillator** by default. No external crystal needed.
Configuration is handled in:
*   `config_common.h`: PID Constants (Kp, Ki, Kd) - Synced with Arduino Uno.
*   `config_internal.h`: Pins and Timings.

## How to Flash (The Tricky Part)
1.  **Programmer**: Use an Arduino Uno as ISP (Example: ArduinoISP sketch).
2.  **Wiring**:
    *   Uno 10 -> ATtiny 1 (Reset) **(Must have 10uF cap on Uno Reset to GND!)**
    *   Uno 11 -> ATtiny 5 (MOSI)
    *   Uno 12 -> ATtiny 6 (MISO)
    *   Uno 13 -> ATtiny 7 (SCK)
    *   5V/GND -> 5V/GND
3.  **Arduino IDE Settings**:
    *   Board: ATtiny25/45/85
    *   Processor: ATtiny85
    *   Clock: **8 MHz (Internal)** -> **Burn Bootloader** first to set fuses!
    *   Programmer: Arduino as ISP
4.  **Upload**: Sketch -> Upload Using Programmer.

*If this process fails often (Device Signature Error), consider switching to the ESP32-C3 version.*
