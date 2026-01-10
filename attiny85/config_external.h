#ifndef CONFIG_ATTINY_EXTERNAL_H
#define CONFIG_ATTINY_EXTERNAL_H

// ATtiny85 configuration - EXTERNAL CRYSTAL VERSION (20MHz)
// For internal oscillator version, use config.h instead

#define F_CPU 20000000UL  // 20MHz external crystal oscillator (PLL mode)

// Pin definitions (ATtiny85 Arduino-style pin numbering)
#define RPM_SENSOR_PIN     3    // Arduino pin 3 (PB3), BLDC Hall sensor input (interrupt capable)
#define PWM_OUTPUT_PIN     0    // Arduino pin 0 (PB0), PWM capable

// Include shared common headers
#include "config_common.h"

// ATtiny85 specific configuration overrides
#define DEFAULT_PULSES_PER_REV 4    // 8-pole BLDC motor with single Hall sensor = 4 pulses per revolution (1 pulse per pole pair)

// Control parameters - Optimized for maximum accuracy on ATtiny85 with external crystal
#define CONTROL_LOOP_HZ     200  // Control loop frequency (200 Hz) - increased for better control with faster clock
#define CONTROL_PERIOD_MS   (1000 / CONTROL_LOOP_HZ)

// Soft-start ramping parameters (ATtiny85 specific)
#define SOFT_START_DURATION_MS  2000  // 2 seconds ramp up time

// External crystal configuration
// Hardware requirements for 20MHz operation:
// - Connect 16MHz crystal between pins 2 and 3 (PB3/PB4)
// - Connect two 22pF capacitors from crystal to ground
// - Fuse settings: Low=0xF1, High=0xDD, Extended=0xFE (set by programmer)

#endif // CONFIG_ATTINY_EXTERNAL_H