#ifndef CONFIG_H
#define CONFIG_H

// ATtiny85 Hardware Configuration
// Refined for 16MHz Internal PLL Operation
#define F_CPU 16000000UL

// Pin Definitions (ATtiny85 Arduino-style pin numbering)
//   PB0 (pin 5) -> PWM Output
//   PB1 (pin 6) -> Sensitivity trim enable (INPUT_PULLUP, pull LOW to enable)
//   PB3 (pin 2) -> RPM Sensor (Hall)
//   PB4 (pin 3) -> Sensitivity pot (ADC2)
#define RPM_SENSOR_PIN        3    // Arduino pin 3 (PB3), Hall sensor input
#define PWM_OUTPUT_PIN        0    // Arduino pin 0 (PB0), PWM output
#define POT_ENABLE_PIN        1    // Arduino pin 1 (PB1)
#define POT_SENSITIVITY_PIN   A2   // Arduino pin 4 (PB4/ADC2)

#define PID_SENSITIVITY_MIN  0.75f
#define PID_SENSITIVITY_MAX  1.25f

// Include shared constants
#include "config_common.h"

// ATtiny Specific Overrides
#define DEFAULT_PULSES_PER_REV 4    // 8-pole BLDC motor (4 pulses per rev)

// Control parameters
#define CONTROL_LOOP_HZ     200
#define CONTROL_PERIOD_MS   (1000 / CONTROL_LOOP_HZ)

// Soft-start ramping parameters
#define SOFT_START_DURATION_MS  1500

#endif // CONFIG_H
