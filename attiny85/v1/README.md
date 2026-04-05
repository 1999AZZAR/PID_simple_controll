# ATtiny85 PID Controller - Version 1

## Overview
Version 1 is the direct production port of the Arduino Uno development code. It runs the exact same logic structure, ensuring that behavior observed during the tuning phase on the Uno is replicated in the final ATtiny85 hardware.

**Key Update (v1.1)**: Now configured to run at **16MHz** (PLL) for improved precision and stability, matching the Uno's clock speed.

## Features
*   **Algorithm**: Standard Floating Point PID (Identical to Uno).
*   **Clock**: 16MHz Internal PLL.
*   **Safety**: Emergency Stop (Power Cut on persistent error).
*   **Start-up**: Non-blocking Soft-Start with Kickstart boost.
*   **Filtering**: Median Filter (Spike Rejection) + EMA (Smoothing).

## Configuration
All settings are located in `config.h`.

### Clock Speed
Ensure your IDE is set to compile for **16MHz (Internal PLL)**.

### Pinout
| Function | ATtiny85 Pin | Physical Pin | Description |
| :--- | :--- | :--- | :--- |
| **PWM Output** | PB0 | Pin 5 | Signal to ESC |
| **Trim enable** | PB1 | Pin 6 | Pull LOW to apply sensitivity pot (internal pullup) |
| **RPM Input** | PB3 | Pin 2 | Hall Sensor Signal |
| **Sensitivity pot** | PB4 (ADC2) | Pin 3 | Scales Kp/Ki/Kd; target RPM stays `DEFAULT_TARGET_RPM` |
| **VCC** | VCC | Pin 8 | 5V Power |
| **GND** | GND | Pin 4 | Ground |

## Flashing Instructions
1.  **Board**: ATtiny25/45/85
2.  **Processor**: ATtiny85
3.  **Clock**: **16 MHz (Internal PLL)**
4.  **Programmer**: Arduino as ISP
5.  **Burn Bootloader**: Run this *once* to set fuses for 16MHz operation.
6.  **Upload**: Upload the sketch.
