#ifndef PCNT_DRIVER_H
#define PCNT_DRIVER_H

#include <Arduino.h>

#ifndef PULSES_PER_REV
#define PULSES_PER_REV      4
#endif

// Fix #1: marked static to avoid ODR violations (multiple-definition linker errors)
// when this header is included in more than one translation unit.
static volatile uint32_t pulse_count = 0;
static unsigned long last_rpm_calc_time = 0;

// IRAM_ATTR is kept; inline is not applicable to ISR handlers registered via
// attachInterrupt — the function address must be stable, so we use static instead.
static void IRAM_ATTR pcnt_isr_handler() {
    pulse_count++;
}

// Fix #1: marked inline to avoid ODR violations
inline void pcnt_init(int gpio_num) {
    pinMode(gpio_num, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(gpio_num), pcnt_isr_handler, RISING);
    last_rpm_calc_time = micros();
}

inline float pcnt_get_rpm() {
    noInterrupts();
    uint32_t count = pulse_count;
    pulse_count = 0;
    unsigned long current_time = micros();
    interrupts();

    unsigned long interval = current_time - last_rpm_calc_time;
    last_rpm_calc_time = current_time;

    if (interval == 0) return 0.0;
    if (count == 0) return 0.0;

    float rpm = ((float)count * 60000000.0) / ((float)interval * PULSES_PER_REV);
    return rpm;
}

#endif // PCNT_DRIVER_H
