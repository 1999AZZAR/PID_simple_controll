#ifndef PCNT_DRIVER_H
#define PCNT_DRIVER_H

#include <Arduino.h>

#ifndef PULSES_PER_REV
#define PULSES_PER_REV      4
#endif

volatile uint32_t pulse_count = 0;
unsigned long last_rpm_calc_time = 0;

void IRAM_ATTR pcnt_isr_handler() {
    pulse_count++;
}

void pcnt_init(int gpio_num) {
    pinMode(gpio_num, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(gpio_num), pcnt_isr_handler, RISING);
    last_rpm_calc_time = micros();
}

float pcnt_get_rpm() {
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
