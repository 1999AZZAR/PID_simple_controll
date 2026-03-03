#ifndef PCNT_DRIVER_H
#define PCNT_DRIVER_H

#include <Arduino.h>

#ifndef PULSES_PER_REV
#define PULSES_PER_REV      4
#endif

#ifndef RPM_SAMPLE_MIN_US
#define RPM_SAMPLE_MIN_US   40000
#endif

static volatile uint32_t pulse_count = 0;
static unsigned long last_rpm_reset_time = 0;
static uint32_t accumulated_pulses = 0;
static float last_computed_rpm = 0.0;

static void IRAM_ATTR pcnt_isr_handler() {
    pulse_count++;
}

inline void pcnt_init(int gpio_num) {
    pinMode(gpio_num, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(gpio_num), pcnt_isr_handler, RISING);
    last_rpm_reset_time = micros();
    accumulated_pulses = 0;
    last_computed_rpm = 0.0;
}

inline float pcnt_get_rpm() {
    noInterrupts();
    uint32_t count = pulse_count;
    pulse_count = 0;
    unsigned long now = micros();
    interrupts();

    accumulated_pulses += count;
    unsigned long interval_us = now - last_rpm_reset_time;

    if (interval_us >= RPM_SAMPLE_MIN_US && interval_us > 0) {
        if (accumulated_pulses > 0) {
            last_computed_rpm = ((float)accumulated_pulses * 60000000.0f) /
                                ((float)interval_us * (float)PULSES_PER_REV);
        } else {
            last_computed_rpm = 0.0f;
        }
        accumulated_pulses = 0;
        last_rpm_reset_time = now;
    }

    return last_computed_rpm;
}

#endif // PCNT_DRIVER_H
