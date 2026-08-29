# Spec — PID Runtime Stability (Bug)

## Overview

Fix intermittent startup and unstable runtime that affects all production variants (ATtiny85 v1/v2/v3, ESP32-C3 control & control_ble). Symptoms are mixed: sometimes motor never starts (requires reset/power-cycle), sometimes takes excessively long to warm up to speed, sometimes runs too fast / overshoots / oscillates. Primary targets are ATtiny85 and ESP32-C3 (small form factor + stability); Arduino Uno is PoC/reference only. Current defaults: `DEFAULT_TARGET_RPM=1440.0`, `DEFAULT_KP=0.150`, `DEFAULT_KI=0.080`, `DEFAULT_KD=0.015`, `PULSES_PER_REV=4`, `RPM_FILTER_SIZE=5`, `SOFT_START_DURATION_MS=1500` / 20 steps.

## Context

- Product: BLDC Motor PID Controller — trapezoidal ESC PWM, single-Hall sensing, shared `pid_common.h`/`rpm_common.h`/`isr_common.h`.
- Hardware: Cameflex KiCad PCB, Hall → PB3 (ATtiny) / GPIO ISR (ESP32-C3, IRAM), PWM → PB0 / LEDC 5 kHz, 5 V→3.3 V divider for ESP32-C3.
- Success metric for this track: **100% start reliability (10/10 cold power cycles start without manual intervention)** at 1440 RPM. Warmup time and overshoot are secondary diagnostics.

## Functional Requirements

- **FR1 — Deterministic cold-start:** Every power-on reaches 1440 RPM without requiring reset, watchdog kick, or manual PWM nudge.
- **FR2 — Soft-start safety + speed:** Ramp must limit current spike yet not stall warmup; target is measurable and bounded (diagnose if 1500 ms/20 steps is too slow or too aggressive per variant).
- **FR3 — Filter correctness:** RPM filter (EMA on Uno, median+EMA on ATtiny v1, moving average on v2, sliding-window on ESP32-C3) must reject Hall noise/aliasing without adding >200 ms lag or masking stall.
- **FR4 — Watchdog / stall failsafe:** ATtiny WDT and ESP32 task WDT must recover from ISR hang or RPM aliasing; stall detection (added in v3/ESP32-C3) must not false-trigger during warmup.
- **FR5 — PID anti-windup & derivative:** Integrator windup on start must be clamped; derivative term must be filtered to avoid noise amplification → overspeed.

## Non-Functional Requirements

- **NFR1 — Reliability:** 10/10 starts on each production variant (ATtiny v1/v2/v3, ESP32-C3 control) under same motor/ESC/supply used for repro.
- **NFR2 — No hardware change:** Fix is firmware/config only; Cameflex PCB and wiring unchanged.
- **NFR3 — CI invariant:** All six firmware variants must compile via `arduino-cli` matrix after fix (`compile.yml`).
- **NFR4 — Minimal footprint impact:** ATtiny must stay within 8 KB / 512 B.

## Acceptance Criteria

- **AC1 — Start reliability:** 10 consecutive cold starts succeed per production variant, no manual reset, documented with Serial/BLE traces.
- **AC2 — Time to stable:** Time to `1440 ± 20 RPM` measured and reported per variant (no hard threshold for this track — data drives tuning).
- **AC3 — No overshoot/oscillation:** Steady-state overshoot <10% and no sustained oscillation after reaching target.
- **AC4 — CI green:** `arduino-cli compile` passes for Uno, ATtiny v1/v2/v3, ESP32-C3 control (and control_ble if present) on CI.
- **AC5 — Diagnostics artifact:** Trace parser script (`scripts/verify_start.py` or equivalent) exists and implements the 10/10 check.

## Out of Scope

- New MCU ports (STM32/RP2040), FOC/sinusoidal control, cloud/mobile app, hardware/PCB redesign, BLE feature additions.

## Repro Notes (from intake)

- Platforms: all production (ATtiny + ESP32-C3) intermittent.
- Symptoms: all three — no-start, slow warmup, too-fast/overspeed.
- Success definition: 100% start reliability.
