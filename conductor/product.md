# BLDC Motor PID Controller — Product Definition

## Summary

High-performance PID control system for precise BLDC motor RPM regulation (default `DEFAULT_TARGET_RPM = 1440.0`) across three microcontroller families: **Arduino Uno** (development/tuning), **ATtiny85 v1/v2/v3** (cost-optimized production), and **ESP32-C3** (RTOS/BLE high-performance). Balances **cost & footprint** (ATtiny85 $1–2, 8-pin DIP, ~8–10 mA) with **precision at 1440 RPM** via anti-windup PID, derivative filtering, and platform-specific RPM filters (EMA / median+EMA / moving average / sliding window). Designed for makers & low-volume builders (1–10 units) using an Arduino-first `tune → ISP flash` workflow. No cloud platform — ESP32-C3 BLE provides local JSON status only.

## Vision

Enable a maker to characterize a BLDC motor on Arduino Uno (Serial Monitor/Plotter + potentiometer live tuning, mode switching) and deploy the *identical* control logic to a minimal ATtiny85 or to a deterministic 200 Hz FreeRTOS ESP32-C3 without re-architecting. Shared modules (`pid_common.h`, `rpm_common.h`, `isr_common.h`, `config_common.h`) guarantee behavioral parity; platform-specific drivers exploit hardware (Timer1 capture, LEDC 5 kHz, PCNT, IRAM ISR) only where they improve stability.

## Target Users

- **Primary:** Makers, educators, low-volume builders prototyping one-off or small-batch devices (1–10 units).
- **Secondary:** Small-run product teams using the Cameflex KiCad PCB for fixed-ESC 3-phase BLDC with single-Hall sensing.
- **Workflow:** `Develop on Uno → record Kp/Ki/Kd → flash ATtiny85 via Arduino-as-ISP (10 µF reset cap) or ESP32-C3 via USB`.

## Goals

1. **RPM Stability at 1440 RPM:** ±1–2% steady-state error, fast settling, no sustained oscillation or overshoot; validated via soft-start ramp and filtered RPM (EMA/median/sliding-window).
2. **Cost & Size Optimized:** ATtiny85 variants fit 8 KB flash / 512 B RAM; PCB-integratable 8-pin DIP; BOM $1–2 for production.
3. **Tunability:** Potentiometer live-tune on Uno without recompilation; fixed-compile constants on ATtiny/ESP32-C3 derived from same tuning session.
4. **Safety & Robustness:** Soft-start protection, emergency stop, watchdog (ATtiny) / task watchdog (ESP32-C3), stall/aliasing failsafe, 5 V→3.3 V divider guidance for ESP32-C3.
5. **Reproducibility:** All six firmware variants (Uno, ATtiny v1/v2/v3, ESP32 control & control_ble) compile in CI (`compile.yml`) with consistent `config.h` interface (`PULSES_PER_REV = N_poles/2`).

## Non-Goals

- **Cloud/App Platform:** No hosted dashboard or Wi-Fi fleet management. BLE JSON notifications (500 ms) and Serial are the only telemetry surfaces.
- **FOC / Sinusoidal Control:** Trapezoidal ESC PWM only; no field-oriented or sensorless vector control this cycle.
- **New MCU Ports:** Freeze at ATmega328P / ATtiny85 / ESP32-C3; no STM32/RP2040/ESP32-S3.

## Constraints

- Motor: 3-phase BLDC with single Hall sensor; Hall → PB3 (ATtiny85) / GPIO ISR (ESP32-C3); PWM → PB0 / LEDC.
- `PULSES_PER_REV = N_poles / 2` (e.g., 8-pole = 4).
- Voltage levels: 5 V logic (Uno/ATtiny) vs 3.3 V (ESP32-C3) requires 2.2 kΩ/3.3 kΩ divider for 5 V Hall.
- Shared codebase must remain Arduino IDE + PlatformIO compatible (`.ino` + `.h`).

## Success Metrics

- RPM stays within ±20 RPM of 1440 RPM under load step.
- PID constants transfer Uno → ATtiny v1 with no re-tune; v2/ESP32 re-tune bounded to <30 min.
- CI: `arduino-cli compile` passes for Uno, ATtiny v1/v2/v3, ESP32-C3 control/control_ble on every push.
- Flash time <2 min/unit via ISP or USB.

## Platforms (in-scope)

| Variant | Role | Notable |
|---------|------|---------|
| Arduino Uno | Dev/Tuning | Serial Plotter, potentiometer live Kp/Ki/Kd, emergency stop |
| ATtiny85 v1 | Production (recommended) | Float PID, direct Uno port, watchdog |
| ATtiny85 v2 | Production (optimized) | Timer1 capture, integer math |
| ATtiny85 v3 | Production (failsafe) | Alias/stall mitigation |
| ESP32-C3 control | High-perf | FreeRTOS 200 Hz, LEDC 5 kHz, sliding window, 160 MHz FPU |
| ESP32-C3 control_ble | IoT | + BLE start/stop, spinlock state, JSON status |
| ESP32-C3 simulator | Tool | Virtual motor physics for offline filter/PID test |
