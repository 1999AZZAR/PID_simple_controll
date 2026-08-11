# ATtiny85 PID Controller - Version 3 (Failsafe)

## Overview

Version 3 keeps v2's hardware-timer measurement (16 us resolution, integer PI,
6-sample moving average) and adds the safety and robustness fixes identified in
review:

*   **Stall / sensor-loss failsafe** - a small state machine (`STATE_SOFTSTART`,
    `STATE_RUN`, `STATE_STALL`). If no sensor pulse arrives within
    `STALL_TIMEOUT_TICKS` (~48 ms, 5x target period) while running, power is
    cut to `PWM_IDLE` and the integral is reset. The controller only re-arms
    via `soft_start()` once pulses resume at an acceptable period, so it can
    never hard-kick back to full PWM after a stall.
*   **Matched anti-windup** - `pid_sum` is clamped to `PID_SUM_LIMIT` (±5000,
    the range the output mapping actually consumes) and the integral is
    clamped to `INTEGRAL_LIMIT` (`= PID_SUM_LIMIT * 100 / K_I`). v2's windup
    limit was ~20x looser than the output rail, so it did not prevent windup.
*   **Seeded moving average** - the 6-sample buffer starts at `TARGET_TICKS`
    instead of zeros, removing the startup under-count transient.
*   **Corrected soft-start handover** - the preloaded integral now uses the
    same mapping as the control loop (`out = PWM_CENTER + pid_sum/48`), so PID
    takeover is seamless. v2's handover math was inconsistent with its own
    mapping and could jump the PWM level.

## Configuration

Configuration is defined at the top of `v3.ino`.

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
3.  **Clock**: 8 MHz internal (factory default, no PLL) - lfuse `0xE2` (`make fuses`)
4.  **Upload**: `make flash` or upload via Arduino IDE

8 MHz internal is the standard clock for this project. It is the chip's factory
default fuse value, requires no PLL lock, and is reliable on every ATtiny85.
The 16 MHz PLL option exists in `hex/` for chips that must run faster, but the
8 MHz builds (`v3_8mhz.hex`) are the recommended ones.

## Difference from v2

*   **v2**: hardware Timer1 measurement, no fault protection. A dead hall
    sensor or a stalled motor froze the PWM at its last value (floored at 45).
*   **v3**: same measurement path plus the stall/sensor-loss failsafe, working
    anti-windup, seeded filter, and a smooth PID handover.
