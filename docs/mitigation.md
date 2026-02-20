# Troubleshooting and Mitigation Guide

This guide provides technical diagnosis and resolution strategies for common issues encountered with the BLDC PID Controller system.

## 1. Motor Issues

### Motor Does Not Start
**Symptoms**: Motor remains stationary despite power and control signals. ESC may beep.
*   **Cause**: Lack of PWM signal, incorrect wiring, or insufficient power.
*   **Diagnosis**:
    1.  Verify PWM signal at microcontroller output (Arduino Pin 9 / ATtiny Pin 5) using an oscilloscope or LED.
    2.  Check ESC connection to power and signal.
    3.  Verify `PWM_MIN_THRESHOLD` in configuration is sufficient to overcome static friction.
*   **Resolution**:
    *   Ensure proper connections.
    *   Increase `PWM_MIN_THRESHOLD` (default: 45).
    *   Calibrate ESC throttle range if supported.

### Unstable RPM / Stalling
**Symptoms**: Motor speed oscillates, erratic behavior, or stalls under load.
*   **Cause**: Electrical noise, poor PID tuning, or weak power supply.
*   **Diagnosis**:
    1.  Monitor voltage at `VIN` or `VCC` during operation; voltage drops indicate insufficient supply.
    2.  Check for "ringing" in RPM readings without load changes.
*   **Resolution**:
    *   **Power**: Use a power supply rated for 2x max current. Add bulk capacitance (1000uF) to power rails.
    *   **Noise**: Add 0.1uF ceramic capacitors to Hall sensor inputs. Use shielded cables.
    *   **Tuning**: Reduce `Kp` and `Ki`. Increase `EMA_ALPHA` for stronger filtering.

### "Kickstart" Failure
**Symptoms**: Motor hums but fails to rotate initially, requiring manual assistance.
*   **Cause**: Static friction exceeds initial torque.
*   **Resolution**:
    *   Enable **Boosted Soft-Start**.
    *   Set `PWM_MIN_THRESHOLD` to 45 or higher.

## 2. RPM Measurement Issues

### Incorrect RPM Readings
**Symptoms**: RPM reading is consistently off by a specific factor (e.g., 2x, 3x).
*   **Cause**: Mismatch between physical motor poles/sensors and software configuration.
*   **Resolution**:
    *   Verify motor pole count.
    *   Update `PULSES_PER_REV` in `config.h`.
    *   Standard 3-Hall 8-pole motor: `PULSES_PER_REV` = 4 (for single wire reading).

### Erratic / Jumping RPM
**Symptoms**: RPM values spike randomly (e.g., 1440 -> 5000 -> 1440).
*   **Cause**: Electrical noise triggering false interrupts.
*   **Resolution**:
    *   **Software**: Enable Median Filter and EMA (Exponential Moving Average).
    *   **Hardware**: Add RC low-pass filter (1kΩ + 10nF) to interrupt pin.

## 3. Electrical & Power Issues

### Power Supply Conflicts (Arduino Uno)
**Symptoms**: Board regulator overheats, erratic logic levels.
*   **Cause**: Simultaneous connection of USB (5V) and external VIN (7-12V).
*   **Resolution**:
    *   **Development**: Use USB power only. Disconnect motor power during code upload if possible.
    *   **Operation**: Use external power via VIN. Disconnect USB.
    *   **Isolation**: Supply logic (Arduino) and power (Motor) from separate sources, sharing only Ground.

### ESP32-C3 Voltage Mismatch
**Symptoms**: ESP32 overheating, pin damage.
*   **Cause**: 5V Hall sensor logic connected directly to 3.3V ESP32 GPIO.
*   **Resolution**:
    *   Use a voltage divider (2kΩ / 3.3kΩ) or logic level shifter between Hall sensor output and ESP32 input.

## 4. ATtiny85 Specific Issues

### Upload Failure
**Symptoms**: "Programmer not responding" or "Invalid device signature".
*   **Cause**: Incorrect ISP wiring or missing Reset capacitor.
*   **Resolution**:
    1.  Ensure 10uF capacitor is between Arduino Uno Reset and Ground.
    2.  Verify SPI connections (MOSI, MISO, SCK, Reset).
    3.  Check that ATtiny85 is clocked at 8MHz (Internal) or 16MHz (PLL) correctly via fuses.

### Watchdog Reset
**Symptoms**: System restarts unexpectedly every few seconds.
*   **Cause**: Main loop execution time exceeds Watchdog Timer (WDT) limit.
*   **Resolution**:
    *   Optimize main loop code.
    *   Ensure `wdt_reset()` is called frequently.

## 5. System Recovery

### Hard Reset
1.  Disconnect all power sources.
2.  Wait 30 seconds for capacitors to discharge.
3.  Reconnect logic power first, then motor power.

### Factory Reset (Software)
*   Re-flash microcontroller with default `config.h` settings to restore known-good PID gains.
