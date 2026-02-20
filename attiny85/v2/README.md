# ATtiny85 PID Controller - Version 2 (Advanced)

## Overview
Version 2 is an optimized implementation that uses **Timer1 Input Capture** for high-precision RPM measurement. Unlike v1 (which measures pulse width via standard interrupts), v2 uses the hardware timer capabilities to measure signal periods with 16µs resolution.

## Features
*   **Precision**: Hardware Timer1 measurement (ticks) instead of `micros()`.
*   **Clock**: 16MHz Internal PLL.
*   **Efficiency**: Lower CPU overhead for measurement.
*   **Control**: Integer-based PID for faster execution.
*   **Stability**: 6-sample Moving Average buffer.

## Configuration
Configuration is defined in `Closed-Loop-Motor-Control.ino`.

### Pinout
| Function | ATtiny85 Pin | Physical Pin | Description |
| :--- | :--- | :--- | :--- |
| **PWM Output** | PB0 | Pin 5 | Signal to ESC |
| **RPM Input** | PB3 | Pin 2 | Hall Sensor Signal |
| **VCC** | VCC | Pin 8 | 5V Power |
| **GND** | GND | Pin 4 | Ground |

## Flashing Instructions
1.  **Board**: ATtiny25/45/85
2.  **Processor**: ATtiny85
3.  **Clock**: **16 MHz (Internal PLL)**
4.  **Burn Bootloader**: Required to enable 16MHz PLL.
5.  **Upload**: Upload the sketch.

## Difference from v1
*   **v1**: Logic clone of Arduino Uno (Float PID, `micros()` measurement). Easier to maintain if you are familiar with Arduino.
*   **v2**: Architecture-specific optimization (Integer PID, Timer1 ticks). Better performance but more complex code.
