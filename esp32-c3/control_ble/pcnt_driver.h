#ifndef PCNT_DRIVER_H
#define PCNT_DRIVER_H

#include <Arduino.h>

// RPM Configuration
#ifndef PULSES_PER_REV
#define PULSES_PER_REV      4
#endif

// Minimum integration window (microseconds) for stable RPM measurement.
// At 1440 RPM @ 4 PPR = 96 pulses/s. 40ms yields ~4 pulses (vs ~0.5 at 5ms).
// Longer window = less quantization noise, slightly higher measurement latency.
#ifndef RPM_SAMPLE_MIN_US
#define RPM_SAMPLE_MIN_US   40000
#endif

// Compute a fresh RPM estimate once this many pulses have accumulated (even
// before the full window elapses). Keeps the PID from running on a stale RPM
// between window boundaries. At 1440 RPM (96 pulses/s) 2 pulses ~= 20ms.
#ifndef RPM_MIN_PULSES
#define RPM_MIN_PULSES      2
#endif

static volatile uint32_t pulse_count = 0;
static unsigned long last_rpm_reset_time = 0;
static uint32_t accumulated_pulses = 0;
static float last_computed_rpm = 0.0;

// Timestamp of the most recent pulse, for stall/sensor-loss detection.
static volatile unsigned long last_pulse_micros = 0;

static void IRAM_ATTR pcnt_isr_handler() {
    pulse_count++;
    last_pulse_micros = micros();
}

inline void pcnt_init(int gpio_num) {
    pinMode(gpio_num, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(gpio_num), pcnt_isr_handler, RISING);
    last_rpm_reset_time = micros();
    last_pulse_micros = last_rpm_reset_time;
    accumulated_pulses = 0;
    last_computed_rpm = 0.0;
}

// Returns RPM. Recomputes whenever the sample window elapses OR enough pulses
// accumulate, so the value tracks the real speed between 40ms boundaries
// instead of returning a stale value to the (faster) control loop.
inline float pcnt_get_rpm() {
    noInterrupts();
    uint32_t count = pulse_count;
    pulse_count = 0;
    unsigned long now = micros();
    interrupts();

    accumulated_pulses += count;
    unsigned long interval_us = now - last_rpm_reset_time;

    if (interval_us >= RPM_SAMPLE_MIN_US || accumulated_pulses >= RPM_MIN_PULSES) {
        if (interval_us > 0) {
            last_computed_rpm = ((float)accumulated_pulses * 60000000.0f) /
                                ((float)interval_us * (float)PULSES_PER_REV);
        }
        accumulated_pulses = 0;
        last_rpm_reset_time = now;
    }

    return last_computed_rpm;
}

// Microseconds since the last detected pulse. Used by the stall failsafe.
inline unsigned long pcnt_pulse_age_us() {
    noInterrupts();
    unsigned long t = last_pulse_micros;
    interrupts();
    return micros() - t;
}

#endif // PCNT_DRIVER_H
