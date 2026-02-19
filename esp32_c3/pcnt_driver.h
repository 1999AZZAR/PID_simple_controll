#ifndef PCNT_DRIVER_H
#define PCNT_DRIVER_H

#include <Arduino.h>

// RPM Configuration
// Adjust based on motor poles (e.g., 8 poles = 4 pulses/rev, 14 poles = 7 pulses/rev)
// This should match the physical hardware. Defaulting to 4 as per previous config.
#ifndef PULSES_PER_REV
#define PULSES_PER_REV      4   
#endif

volatile uint32_t pulse_count = 0;
unsigned long last_rpm_calc_time = 0;

// Interrupt Service Routine for Pulse Counting
void IRAM_ATTR pcnt_isr_handler() {
    pulse_count++;
}

void pcnt_init(int gpio_num) {
    // Set up the input pin with internal pullup
    pinMode(gpio_num, INPUT_PULLUP);
    
    // Attach interrupt on Rising edge (configure as needed: RISING, FALLING, CHANGE)
    attachInterrupt(digitalPinToInterrupt(gpio_num), pcnt_isr_handler, RISING);
    
    last_rpm_calc_time = micros();
}

float pcnt_get_rpm() {
    // Critical section to read and reset counter atomically
    noInterrupts();
    uint32_t count = pulse_count;
    pulse_count = 0;
    unsigned long current_time = micros();
    interrupts();
    
    unsigned long interval = current_time - last_rpm_calc_time;
    last_rpm_calc_time = current_time;
    
    if (interval == 0) return 0.0;
    if (count == 0) return 0.0;
    
    // RPM = (Pulses * 60,000,000) / (Interval_us * PPR)
    float rpm = ((float)count * 60000000.0) / ((float)interval * PULSES_PER_REV);
    
    return rpm;
}

#endif // PCNT_DRIVER_H
