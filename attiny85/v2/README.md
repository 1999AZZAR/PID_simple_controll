# ATtiny85 Closed-Loop Motor Control

A Proportional-Integral (PI) controller implementation maintaining a precise motor speed using Hall sensor feedback.

## Description
This project maintains a DC motor at exactly **1440 RPM** using a closed-loop feedback system.
*   **Sensor**: Reads pulses from a Hall effect sensor (4 pulses per revolution) on **PB3**.
*   **Actuator**: Adjusts PWM duty cycle on **PB0** to correct speed deviations.
*   **Algorithm**: Measures the exact time period between sensor pulses (in microseconds) and uses a PI algorithm to adjust power.

## Hardware Connection

| Component | ATtiny85 Pin | Physical Pin | Notes |
|---|---|---|---|
| **Motor Driver (PWM)** | PB0 | Pin 5 | Connect to MOSFET Gate or Driver Input |
| **Hall Sensor** | PB3 | Pin 2 | Connect Signal Output (Open Drain + Pull-up) |
| **Power** | VCC | Pin 8 | 5V (Stable) |
| **Ground** | GND | Pin 4 | Common Ground |

## Technical Highlights

### 1. Precision Timing
Instead of counting pulses over time (which is slow and imprecise at low speeds), this code measures the **period** between pulses using Timer1.
*   **Clock**: 16MHz (via PLL) for higher resolution.
*   **Resolution**: 16µs per tick.
*   **Target**: 1440 RPM = 96 Hz = ~10.4ms period = ~651 ticks.

### 2. Advanced Control Features
*   **PI Control**: Uses Proportional and Integral terms to maintain exact speed under varying loads.
*   **Moving Average Filter**: A 4-sample buffer smoothes out sensor jitter and prevents oscillation.
*   **Adaptive Soft-Start**: Linearly ramps power from 25% to target over 1.5s to prevent mechanical stress. The logic monitors speed during startup and seamlessly hands over to the PID controller as soon as 1440 RPM is reached, preventing overshoot.

## Build and Flash

**Important**: This example runs at 16MHz. You must update the fuses.

```bash
# 1. Set Fuses for 16MHz PLL
make fuses

# 2. Compile and Upload
make flash
```
