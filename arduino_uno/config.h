#ifndef CONFIG_H
#define CONFIG_H

// Include necessary libraries
#include <Arduino.h>
#include <EEPROM.h>
#include <avr/wdt.h>

// Pin definitions
#define RPM_SENSOR_PIN      2   // Interrupt pin for BLDC Hall sensor (any Hall wire from motor)
#define PWM_OUTPUT_PIN      9   // PWM output to ESC
#define MODE_SWITCH_PIN     3   // Digital input for mode selection
#define POT_TARGET_RPM      A0  // Potentiometer for target RPM
#define POT_KP              A1  // Potentiometer for Kp gain
#define POT_KI              A2  // Potentiometer for Ki gain
#define POT_KD              A3  // Potentiometer for Kd gain
#define POT_PULSES_PER_REV  A4  // Potentiometer for pulses per revolution

// Control parameters
#define CONTROL_LOOP_HZ     100 // Control loop frequency (100 Hz)
#define CONTROL_PERIOD_MS   (1000 / CONTROL_LOOP_HZ)

// RPM calculation parameters
#define PULSES_PER_REV      18   // Default number of pulses per revolution
#define RPM_CALC_INTERVAL   100 // RPM calculation interval in ms
#define MIN_PULSE_WIDTH_US  100 // Minimum pulse width to reject EMI spikes (100-500us)

// PID limits
#define PID_OUTPUT_MIN      -255 // Minimum PID output
#define PID_OUTPUT_MAX      255  // Maximum PID output
#define INTEGRAL_WINDUP_MIN -100 // Anti-windup integral minimum
#define INTEGRAL_WINDUP_MAX 100  // Anti-windup integral maximum

// Production mode default values (tune these during testing)
#define PRODUCTION_TARGET_RPM 1440.0
#define PRODUCTION_KP         0.5
#define PRODUCTION_KI         0.1
#define PRODUCTION_KD         0.01

// Serial command buffer
#define SERIAL_BUFFER_SIZE    64

// Debug mode settings
#define DEBUG_MODE_ENABLED     false  // Set to true for debug output
#define DEBUG_INTERVAL_MS      1000   // Debug print interval

// EEPROM addresses for storing parameters
#define EEPROM_TARGET_RPM_ADDR     0
#define EEPROM_KP_ADDR             4
#define EEPROM_KI_ADDR             8
#define EEPROM_KD_ADDR             12
#define EEPROM_PULSES_PER_REV_ADDR 16

// Soft-start ramping parameters
#define SOFT_START_DURATION_MS  2000  // 2 seconds ramp up time
#define SOFT_START_STEPS       20     // Number of ramp steps

// Safety parameters
#define WATCHDOG_ENABLED           true   // Enable watchdog timer for hang protection
#define WATCHDOG_TIMEOUT           WDTO_4S  // 4-second watchdog timeout
#define EMERGENCY_STOP_ENABLED     true   // Enable emergency stop feature (now enabled after memory optimization)
#define EMERGENCY_STOP_TIMEOUT_MS  5000  // Stop PWM if no pulses received for 5 seconds

#endif // CONFIG_H
