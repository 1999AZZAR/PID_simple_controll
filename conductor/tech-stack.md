# Tech Stack — BLDC Motor PID Controller

## Overview

Brownfield embedded product. **Primary targets: ATtiny85 (minimal form factor) and ESP32-C3 (small form factor + maximum stability).** Arduino Uno is **PoC/code validation only** — used to prove PID logic and tune constants, not deployed.

## Languages

- **C/C++ (Arduino Wiring):** All firmware in `.ino` + shared `.h` modules (`pid_common.h`, `rpm_common.h`, `isr_common.h`, `config_common.h`, `config.h`). Arduino IDE compatible, PlatformIO compatible.

## Microcontrollers & Hardware

| Target | MCU | Clock | Role | Form Factor |
|--------|-----|-------|------|-------------|
| ATtiny85 v1/v2/v3 | ATtiny85 (AVR 8-bit) | 16 MHz PLL internal | **Production — minimal size** | 8-pin DIP/SOIC, ~1 cm², 8–10 mA |
| ESP32-C3 control / control_ble / simulator | ESP32-C3 (RISC-V 32-bit, FPU) | 160 MHz | **Production — small + stable** | Module ~18×20 mm, ~60–80 mA |
| Arduino Uno | ATmega328P | 16 MHz | **PoC only** | Board 68×53 mm, ~50 mA — not for deployment |

- **Motor HW:** 3-phase BLDC + single Hall sensor + ESC (PWM in). Hall → PB3 (ATtiny) / GPIO ISR (ESP32-C3) with `micros()` / Timer1 capture / PCNT; PWM → PB0 / LEDC 5 kHz. 5 V Hall requires 2.2 kΩ/3.3 kΩ divider for ESP32-C3 3.3 V inputs.

## Frameworks & Libraries

- **Arduino Core:** `arduino:avr` (Uno), `attiny:avr` Damellis (ATtinyX5, `cpu=attiny85,clock=internal16`), `esp32:esp32` Espressif (ESP32-C3).
- **ESP32-C3 specifics:** FreeRTOS (200 Hz PID task, highest priority), LEDC hardware PWM, IRAM-resident ISR, sliding-window filter, NimBLE/BLE for `control_ble` (JSON status 500 ms, spinlock shared state).
- **Common filtering:** EMA / median+EMA / moving-average / sliding-window per variant.

## Build & Tooling

- **Build:** `arduino-cli compile --fqbn <target>` per variant (`arduino:avr:uno`, `attiny:avr:ATtinyX5:cpu=attiny85,clock=internal16`, `esp32:esp32:esp32c3:CDCOnBoot=default`). Helpers: `attiny85/build_all.sh`, `compile_test.sh`, `flash.sh` (Arduino-as-ISP, 10 µF reset cap).
- **Artifacts:** `compiled/` hex + `DDMMYYYY_<repo>.zip` release.
- **PCB:** KiCad (`PCB/cameflex/cameflex.kicad_*`, `fp-info-cache`); Cameflex board is deployment carrier for ATtiny/ESP32-C3.

## CI/CD

- **GitHub Actions** `.github/workflows/compile.yml`: `setup-arduino-cli@v1` → `config add board_manager.additional_urls` (Damellis + Espressif) → `core install avr/attiny/esp32` → compile Uno + ATtiny v1/v2/v3 + ESP32-C3 control → list artifacts → memory usage check → ZIP + auto-release (retry on 502).
- **Triggers:** pushes to `main`/`master`, tags `v*.*.*`, PRs.

## Quality & Style

- `.clang-format` (project root), `.pre-commit-config.yaml` (linters/formatters), `CONTRIBUTING.md` guidelines.
- No JS/TS/Python runtime; docs built via `docs/_config.yml` (GitHub Pages).

## Design Priorities

1. **Small form factor first** — ATtiny85 and ESP32-C3 modules only.
2. **Stability at 1440 RPM** — 200 Hz deterministic loop (ESP32-C3) + watchdog/stall failsafe; Uno is reference for tuning, not stability benchmark.
3. **Portability of PID constants** — Uno-tuned `DEFAULT_KP/KI/KD` copies to ATtiny/ESP32 with minimal re-tune.
