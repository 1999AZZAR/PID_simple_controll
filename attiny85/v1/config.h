#ifndef CONFIG_H
#define CONFIG_H

// ATtiny85 Hardware Configuration
// Refined for 16MHz Internal PLL Operation
#define F_CPU 16000000UL

// Pin Definitions (ATtiny85 Arduino-style pin numbering)
#define RPM_SENSOR_PIN     3    // Arduino pin 3 (PB3), Hall sensor input
#define PWM_OUTPUT_PIN     0    // Arduino pin 0 (PB0), PWM output

// Include shared constants
#include "config_common.h"

// ATtiny Specific Overrides
#define DEFAULT_PULSES_PER_REV 4    // 8-pole BLDC motor (4 pulses per rev)

// Control parameters - Optimized for maximum accuracy on ATtiny85
#define CONTROL_LOOP_HZ     200     // Control loop frequency (200 Hz) - Matches Arduino Uno
#define CONTROL_PERIOD_MS   (1000 / CONTROL_LOOP_HZ)

// Soft-start ramping parameters
#define SOFT_START_DURATION_MS  1500  // 1.5 seconds ramp up time (Matches Arduino Uno)

#endif // CONFIG_H
