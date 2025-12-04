#ifndef CONFIG_H
#define CONFIG_H

// Include necessary libraries
#include <Arduino.h>

// Pin definitions
#define RPM_SENSOR_PIN      2   // Interrupt pin for BLDC Hall sensor (any Hall wire from motor)
#define PWM_OUTPUT_PIN      9   // PWM output to ESC
#define MODE_SWITCH_PIN     3   // Digital input for mode selection
#define POT_PULSES_PER_REV  A0  // Potentiometer for pulses per revolution
#define POT_KP              A1  // Potentiometer for Kp gain
#define POT_KI              A2  // Potentiometer for Ki gain
#define POT_KD              A3  // Potentiometer for Kd gain

// Control parameters - Optimized for maximum accuracy and responsiveness
#define CONTROL_LOOP_HZ     200 // Control loop frequency (200 Hz) - higher frequency for better control
#define CONTROL_PERIOD_MS   (1000 / CONTROL_LOOP_HZ)

// RPM calculation parameters
#define DEFAULT_PULSES_PER_REV 24   // Default number of pulses per revolution got from 4 cycles×6 steps=24 state changes per revolution.
#define RPM_CALC_INTERVAL   10  // RPM calculation interval (10ms) - faster updates for maximum accuracy
#define MIN_PULSE_WIDTH_US  50   // Minimum pulse width (50μs) - optimized for fast response

// Serial communication parameters
#define SERIAL_SEND_INTERVAL 100  // Send status data every 100ms (10Hz)

// PID limits - Optimized for low-speed control
#define PID_OUTPUT_MIN      -1000 // Minimum PID output (expanded for low speeds)
#define PID_OUTPUT_MAX      1000  // Maximum PID output (expanded for low speeds)
#define INTEGRAL_WINDUP_MIN -200  // Anti-windup integral minimum (expanded)
#define INTEGRAL_WINDUP_MAX 200   // Anti-windup integral maximum (expanded)

// Default PID parameters (can be adjusted via Python GUI)
#define DEFAULT_TARGET_RPM 1440.0  // Default target RPM
#define DEFAULT_KP         3.25    // Default proportional gain - balanced for 100Hz loop
#define DEFAULT_KI         0.0320   // Default integral gain - conservative for stability
#define DEFAULT_KD         0.001   // Default derivative gain - minimal for noise rejection

// Soft-start ramping parameters
#define SOFT_START_DURATION_MS  2000  // 2 seconds ramp up time
#define SOFT_START_STEPS       20     // Number of ramp steps

#endif // CONFIG_H
