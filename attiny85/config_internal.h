#ifndef CONFIG_ATTINY_H
#define CONFIG_ATTINY_H

// ATtiny85 configuration - INTERNAL OSCILLATOR VERSION (8MHz)
// For external crystal version, use config_external.h instead

#define F_CPU 8000000UL  // 8MHz internal oscillator

// Pin definitions (ATtiny85 Arduino-style pin numbering)
#define RPM_SENSOR_PIN     3    // Arduino pin 3 (PB3), BLDC Hall sensor input (interrupt capable)
#define PWM_OUTPUT_PIN     0    // Arduino pin 0 (PB0), PWM capable

// Include shared common headers
#include "config_common.h"

// ATtiny85 specific configuration overrides
#define DEFAULT_PULSES_PER_REV 4    // 8-pole BLDC motor (4 pulses per rev)

// Control parameters - Optimized for maximum accuracy on ATtiny85
#define CONTROL_LOOP_HZ     200     // Control loop frequency (200 Hz) - Matches Arduino Uno
#define CONTROL_PERIOD_MS   (1000 / CONTROL_LOOP_HZ)

// Soft-start ramping parameters (ATtiny85 specific)
#define SOFT_START_DURATION_MS  1500  // 1.5 seconds ramp up time (Matches Arduino Uno)

#endif // CONFIG_ATTINY_H
