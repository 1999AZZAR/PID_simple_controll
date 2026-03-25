#ifndef CONFIG_H
#define CONFIG_H

// Include necessary libraries
#include <Arduino.h>

// Include shared common headers
#include "config_common.h"

// Pin definitions
#define RPM_SENSOR_PIN      2   // Interrupt pin for BLDC Hall sensor (any Hall wire from motor)
#define PWM_OUTPUT_PIN      9   // PWM output to ESC
#define POT_ENABLE_PIN      3   // Digital input - Pull LOW to enable pot target mode (INPUT_PULLUP)
#define POT_TARGET_RPM_PIN  A4  // Potentiometer for target RPM (1000-3000)
#define POT_PULSES_PER_REV  A0  // Potentiometer for pulses per revolution (tuning mode)
#define POT_KP              A1  // Potentiometer for Kp gain (tuning mode)
#define POT_KI              A2  // Potentiometer for Ki gain (tuning mode)
#define POT_KD              A3  // Potentiometer for Kd gain (tuning mode)

// Target RPM Range (when pot is enabled)
#define TARGET_RPM_MIN  1000
#define TARGET_RPM_MAX  3000

// Arduino Uno specific configuration overrides
#define DEFAULT_PULSES_PER_REV 4    // 8-pole BLDC motor with single Hall sensor = 4 pulses per revolution (1 pulse per pole pair)

// Serial communication parameters
#define SERIAL_SEND_INTERVAL 100  // Send status data every 100ms (10Hz)

// Soft-start ramping parameters
#define SOFT_START_DURATION_MS  1500  // 1.5 seconds ramp up time
#define SOFT_START_STEPS       20     // Number of ramp steps

#endif // CONFIG_H
