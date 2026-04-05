# ESP32-C3 BLDC PID Controller (Production)

**High-performance PID controller** for BLDC motors leveraging the ESP32-C3's hardware peripherals and FreeRTOS for ultra-stable, real-time control.

## Table of Contents

- [Overview](#overview)
- [Quick Start](#quick-start)
- [Project Structure](#project-structure)
- [Hardware Setup](#hardware-setup)
- [Software Architecture](#software-architecture)
- [Configuration](#configuration)
- [Features](#features)
- [Troubleshooting](#troubleshooting)

## Overview

The ESP32-C3 version is designed for production environments where stability and performance are critical. It utilizes a dedicated high-priority FreeRTOS task to ensure a consistent 200Hz control loop, independent of other system activities.

### Key Features

- **200Hz Control Loop**: High-frequency PID updates via FreeRTOS task.
- **Hardware-Accelerated PWM**: Uses the ESP32 LEDC peripheral for smooth 5kHz motor control.
- **Interrupt-Driven RPM**: High-precision pulse counting using IRAM-resident ISR.
- **Soft-Start Logic**: 1.5-second power ramp to prevent current spikes and mechanical stress.
- **EMA Filtering**: Exponential Moving Average filtering for stable RPM readings.
- **FPU Utilization**: Full floating-point PID calculations for maximum precision.

## Quick Start

1. **Hardware Setup**: Connect the RPM sensor to GPIO 0 (use a voltage divider!) and the ESC to GPIO 1.
2. **Configure**: Review `config.h` and `config_common.h` for motor-specific parameters (PPR, PID gains).
3. **Upload**: Flash the `control.ino` sketch using the Arduino IDE or ESP-IDF (with Arduino component).
4. **Monitor**: Open the Serial Monitor at 115200 baud to verify startup.

## Project Structure

```
esp32-c3/control/
├── control.ino        # Main entry point and FreeRTOS task management
├── config.h           # Hardware-specific pin and motor settings
├── config_common.h    # Shared PID and control loop constants
├── ledc_driver.h      # ESP32 LEDC PWM driver implementation
├── pcnt_driver.h      # Interrupt-based pulse counting logic
├── pid_common.h       # Floating-point PID algorithm
├── rpm_common.h       # RPM filtering (EMA) utilities
└── README.md          # This documentation
```

## Hardware Setup

### Required Components
- **ESP32-C3 SuperMini** (or any ESP32-C3 dev board)
- **BLDC Motor** (e.g., 8-pole motor)
- **Electronic Speed Controller (ESC)**
- **Voltage Divider**: Necessary for RPM input (5V Hall sensor signal -> 3.3V ESP32 GPIO).
  - **R1 (2.2kΩ)**: Connects between the 5V Hall Sensor Signal and GPIO 0.
  - **R2 (3.3kΩ)**: Connects between GPIO 0 and Ground.
  
  **Wiring Diagram:**
  ```text
  5V Hall Signal ----[ R1: 2.2kΩ ]---- GPIO 0
                               |
                               +----[ R2: 3.3kΩ ]---- GND
  ```

### Pin Connections

| Component | ESP32-C3 Pin | Description |
|-----------|--------------|-------------|
| **RPM Input** | GPIO 0 | Pulse signal from motor Hall sensor (Requires 5V->3.3V divider) |
| **PWM Output** | GPIO 1 | PWM control signal to the ESC |
| **Sensitivity pot** | GPIO 2 (ADC) | Optional: scales Kp/Ki/Kd when GPIO 3 is LOW |
| **Trim enable** | GPIO 3 | Optional: LOW = apply pot; HIGH = gain scale 1.0 |
| **Power** | 5V / VIN | 5V Power supply |
| **Ground** | GND | Common ground |

⚠️ **WARNING:** The ESP32-C3 GPIOs are **not 5V tolerant**. You MUST use a voltage divider (e.g., 2.2kΩ/3.3kΩ) if your Hall sensor outputs a 5V signal.

## Software Architecture

### Control Loop
The controller runs in a dedicated FreeRTOS task (`ControlLoop`) with the highest priority (`configMAX_PRIORITIES - 1`). This ensures that the 5ms (200Hz) timing is strictly maintained even if the main `loop()` or Serial communication becomes busy.

### Drivers
- **`pcnt_driver.h`**: Manages pulse counting via a falling/rising edge interrupt. It calculates RPM based on the microsecond interval between reads to ensure accuracy across the entire speed range.
- **`ledc_driver.h`**: Configures the ESP32 LEDC peripheral to generate a 5kHz PWM signal with 8-bit resolution, providing silent and smooth motor operation.

## Configuration

Settings are split between `config.h` (Hardware/Motor) and `config_common.h` (PID/Logic).

### Key Constants (`config.h`)
- `PULSES_PER_REV`: Set to 4 for an 8-pole motor (1 pulse per 2 poles).
- `SOFT_START_DURATION_MS`: 1500ms ramp time.
- `POT_ENABLE_PIN` / `POT_SENSITIVITY_PIN` / `PID_SENSITIVITY_MIN` / `PID_SENSITIVITY_MAX`: optional gain trim (target RPM stays in `config_common.h`).

### PID Constants (`config_common.h`)
- `DEFAULT_TARGET_RPM`: 1440.0
- `DEFAULT_KP`: 0.150
- `DEFAULT_KI`: 0.080
- `DEFAULT_KD`: 0.015

## Features

### Soft-Start (Kickstart)
Upon power-up, the controller implements a "Kickstart" ramp. It begins at `PWM_MIN_THRESHOLD` and linearly ramps up to the PID-calculated value over 1.5 seconds. This prevents the motor from "stuttering" during initial torque application.

### RPM Filtering
The raw RPM calculated from pulses often contains jitter. The system uses an Exponential Moving Average (EMA) to smooth the input to the PID loop without introducing significant lag.

## Troubleshooting

### No RPM Signal
- Check voltage divider connections.
- Ensure the Hall sensor is powered correctly.
- Verify `PULSES_PER_REV` matches your motor.

### Motor Stutters on Start
- Increase `PWM_MIN_THRESHOLD` in `config_common.h`.
- Extend `SOFT_START_DURATION_MS`.

### Unstable Speed
- Tuning required: Reduce `DEFAULT_KP` if the motor oscillates.
- Increase `EMA_ALPHA` for faster response, or decrease it for more stability.
- Check for electrical noise on the RPM input line (consider a decoupling capacitor).
