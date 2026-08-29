# Product Guidelines — BLDC Motor PID Controller

## Brand & Voice

- **Practical over polished:** Concise, hardware-grounded prose. No marketing fluff. Assume reader has soldering iron and multimeter.
- **Safety-first:** Every wiring/voltage instruction pairs with a ⚠️ callout (5 V→3.3 V divider, common-ground only, prop removal, 10 µF ISP cap).
- **Show, don't describe:** Pin maps, `config.h` snippets, and Serial Plotter screenshots beat paragraphs.
- **Audience:** Makers & low-volume builders (1–10 units). Write at technician level — no EE degree assumed, but no hand-holding on Arduino IDE install.

## Documentation Principles

1. **Single source of truth:** `README.md` = overview; `docs/howto.md` = wiring; `docs/mitigation.md` = troubleshooting; platform `README.md` = per-variant steps. No duplication.
2. **Copy-pasteable configs:** Every tunable (`PULSES_PER_REV`, `DEFAULT_TARGET_RPM`, `DEFAULT_KP/KI/KD`, `PWM_MIN_THRESHOLD`) shown as compilable snippet.
3. **Visual hierarchy:** Tables for comparison/migration; code fences for configs; checklists for hardware assembly.
4. **CI as proof:** "If it compiles in `compile.yml`, it's documented." Link CI badge to build matrix.

## UX Principles (Hardware + Firmware)

- **Tune on Uno, deploy on ATtiny:** Potentiometer live-tune → fixed constants. No re-tune for v1; bounded re-tune for v2/ESP32-C3.
- **Immediate feedback:** Serial Monitor/Plotter on Uno/ESP32-C3 shows RPM, error, PWM every loop; BLE JSON every 500 ms on `control_ble`.
- **Fail safely:** Soft-start ramp on every start; emergency stop pin halts PWM; watchdog/stall failsafe recovers without power cycle.
- **Minimal surprise:** ATtiny v1 behaves identically to Uno (float PID). v2/v3 and ESP32-C3 differences are explicitly tabled in `COMPARISON.md`.
- **Reversible actions:** All wiring is through-hole/DIP; no destructive mods. Simulator variant allows risk-free PID experimentation.

## Content Standards

- Language: English, units in SI (RPM, ms, mA, kΩ), pin names match silkscreen (`PB3/PB0`).
- Images: `docs/` + `PCB/data/` contain wiring photos, KiCad renders, and scope captures; keep filenames versioned.
- Length: Each guide <800 words before tables/code; troubleshooting entries are `Issue | Cause | Fix` rows.

## Non-Goals (Brand)

- No lifestyle branding, no cloud portal, no mobile app distribution — BLE uses generic nRF Connect/LightBlue.
- No dark-pattern upsells; cost tables show $1.50 ATtiny vs $4 ESP32 honestly.
