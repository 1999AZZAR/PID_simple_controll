#ifndef CONFIG_H
#define CONFIG_H

// Include necessary libraries
#include <Arduino.h>

// Include shared common headers
#include "config_common.h"

// Pin definitions
#define RPM_SENSOR_PIN 2       // Interrupt pin for BLDC Hall sensor (any Hall wire from motor)
#define PWM_OUTPUT_PIN 9       // PWM output to ESC
#define POT_ENABLE_PIN 3       // Pull LOW to enable PID sensitivity trim (INPUT_PULLUP)
#define POT_SENSITIVITY_PIN A4 // Trims Kp/Ki/Kd scale; target RPM stays DEFAULT_TARGET_RPM

#define PID_SENSITIVITY_MIN 0.75f
#define PID_SENSITIVITY_MAX 1.25f

// Arduino Uno specific configuration overrides
#define DEFAULT_PULSES_PER_REV                                                                     \
    4 // 8-pole BLDC motor with single Hall sensor = 4 pulses per revolution (1 pulse per pole pair)

// Serial communication parameters
#define SERIAL_SEND_INTERVAL 100 // Send status data every 100ms (10Hz)

// Soft-start ramping parameters
#define SOFT_START_DURATION_MS 1500 // 1.5 seconds ramp up time
#define SOFT_START_STEPS 20         // Number of ramp steps

#endif // CONFIG_H
