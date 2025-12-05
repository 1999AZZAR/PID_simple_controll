#ifndef CONFIG_H
#define CONFIG_H

// Include necessary libraries
#include <Arduino.h>

// Include shared common headers
#include "config_common.h"

// Pin definitions
#define RPM_SENSOR_PIN      2   // Interrupt pin for BLDC Hall sensor (any Hall wire from motor)
#define PWM_OUTPUT_PIN      9   // PWM output to ESC
#define MODE_SWITCH_PIN     3   // Digital input for mode selection
#define POT_PULSES_PER_REV  A0  // Potentiometer for pulses per revolution
#define POT_KP              A1  // Potentiometer for Kp gain
#define POT_KI              A2  // Potentiometer for Ki gain
#define POT_KD              A3  // Potentiometer for Kd gain

// Arduino Uno specific configuration overrides
#define DEFAULT_PULSES_PER_REV 4    // 8-pole BLDC motor with single Hall sensor = 4 pulses per revolution (1 pulse per pole pair)

// Serial communication parameters (Arduino Uno specific)
#define SERIAL_SEND_INTERVAL 100  // Send status data every 100ms (10Hz)

// Pin definitions (Arduino Uno specific)
#define MODE_SWITCH_PIN     3   // Digital input for mode selection
#define POT_PULSES_PER_REV  A0  // Potentiometer for pulses per revolution
#define POT_KP              A1  // Potentiometer for Kp gain
#define POT_KI              A2  // Potentiometer for Ki gain
#define POT_KD              A3  // Potentiometer for Kd gain

// Soft-start ramping parameters
#define SOFT_START_DURATION_MS  5000  // 5 seconds ramp up time
#define SOFT_START_STEPS       20     // Number of ramp steps

#endif // CONFIG_H
