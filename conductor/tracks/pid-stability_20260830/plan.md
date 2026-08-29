# Plan — PID Runtime Stability

## Phase 1 — Diagnosis & Repro (Red Phase)

- [ ] Task: Instrument unified debug trace
  - [ ] Add conditional `SERIAL_DEBUG` trace (RPM raw/filtered, PWM, dt, ISR count, soft-start step, WDT status) to ATtiny v1/v2/v3 and ESP32-C3 control/control_ble without exceeding ATtiny RAM
  - [ ] Verify trace builds with and without `SERIAL_DEBUG` (CI-safe)
- [ ] Task: Capture 10 cold-start traces per production variant
  - [ ] Define harness: same motor/ESC/supply, power-cycle via controlled off-time, log via Uno-serial or ESP32 USB
  - [ ] Record: time to 1440±20 RPM, overshoot, stall events, WDT resets
- [ ] Task: Write failing repro check (RED)
  - [ ] Create `scripts/verify_start.py` — parses trace CSV/serial log, asserts 10/10 starts within bounded warmup and no stall/no-reset
  - [ ] Run on current firmware; confirm failure (intermittent no-start / slow warmup / overspeed reproduces)
- [ ] Task: Phase Verification & Checkpoint (Refer to workflow.md)
  - [ ] Traces archived under `conductor/tracks/pid-stability_20260830/evidence/`
  - [ ] Repro script demonstrates RED before fix

## Phase 2 — Stabilization Fixes (Green Phase)

- [ ] Task: Audit PID, filter, and ISR against gathered traces
  - [ ] Review `pid_common.h` (anti-windup, integral clamp, derivative filter), `rpm_common.h`/`isr_common.h` (Hall timing, aliasing, Timer1 capture vs micros, PCNT/LEDC ISR)
  - [ ] Compare filter lag: EMA vs median+EMA vs moving-average vs sliding-window (size 5) vs warmup delay
  - [ ] Check `PWM_MIN_THRESHOLD`, `SOFT_START_DURATION_MS=1500 / STEPS=20` per variant
- [ ] Task: Fix soft-start + ISR timing (GREEN)
  - [ ] Fix Hall ISR debounce/aliasing (stall failsafe in v3/ESP32-C3 `rpm_common.h` threshold, min-pulse validation)
  - [ ] Tune soft-start per variant if trace shows stall (too aggressive) or slow warmup (too conservative); keep 1440 RPM target fixed, pot only scales Kp/Ki/Kd
  - [ ] Validate `PULSES_PER_REV=4` RPM calc vs actual pulses
- [ ] Task: Harden watchdog & failsafe recovery
  - [ ] ATtiny: verify WDT enable/pet timing doesn't conflict with soft-start delay; ESP32-C3: task WDT 5 s, spinlock safe state
  - [ ] Emergency-stop path returns to soft-start rather than abrupt PWM jump
- [ ] Task: Tune PID filter constants if needed (bounded change)
  - [ ] Keep DEFAULT_KP=0.150/KI=0.080/KD=0.015 as baseline; any change documented with before/after traces
- [ ] Task: Phase Verification & Checkpoint (Refer to workflow.md)
  - [ ] Re-compile matrix passes; per-variant RAM/flash within limits

## Phase 3 — Verification, Hardening & Docs (Refactor + Coverage)

- [ ] Task: Re-run repro script to GREEN
  - [ ] 10/10 cold starts pass per production variant; time-to-1440±20 RPM reported
  - [ ] No overshoot >10% or sustained oscillation at steady state
- [ ] Task: CI & build verification
  - [ ] `arduino-cli compile` for Uno + ATtiny v1/v2/v3 + ESP32-C3 control (& control_ble/simulator if applicable) green in GitHub Actions and locally via `attiny85/build_all.sh`
- [ ] Task: Update documentation
  - [ ] `docs/mitigation.md` — add symptom → cause → fix entry for this class of instability
  - [ ] `docs/COMPARISON.md` — update warmup/stability notes per variant if tuning changed
  - [ ] Platform READMEs — clarify 1440 RPM stability guidance
- [ ] Task: Phase Verification & Checkpoint (Refer to workflow.md)
  - [ ] Evidence (traces, script output, CI logs) linked in track `evidence/` folder
  - [ ] Ready for conductor review

## Notes

- Follow workflow.md TDD: RED (failing verify script) → GREEN (minimal firmware fix) → Refactor (cleanup, no behavior change) → Coverage (CI + reliability runs).
- No hardware/PCB changes; firmware only.
