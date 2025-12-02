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

// Control parameters - Optimized for real-time performance
#define CONTROL_LOOP_HZ     100 // Control loop frequency (100 Hz) - optimal for BLDC control
#define CONTROL_PERIOD_MS   (1000 / CONTROL_LOOP_HZ)

// RPM calculation parameters
#define PULSES_PER_REV      18   // Default number of pulses per revolution
#define RPM_CALC_INTERVAL   25  // RPM calculation interval (25ms) - faster updates for smooth control
#define MIN_PULSE_WIDTH_US  50  // Minimum pulse width (50μs) - reduced for better responsiveness

// PID limits - Expanded range for higher resolution control
#define PID_OUTPUT_MIN      -500 // Minimum PID output
#define PID_OUTPUT_MAX      500  // Maximum PID output
#define INTEGRAL_WINDUP_MIN -100  // Anti-windup integral minimum
#define INTEGRAL_WINDUP_MAX 100   // Anti-windup integral maximum

// Production mode default values - Optimized for 100Hz control loop
#define PRODUCTION_TARGET_RPM 1440.0  // Original target RPM
#define PRODUCTION_KP         0.25   // Proportional gain - balanced for 100Hz loop
#define PRODUCTION_KI         0.015  // Integral gain - conservative for stability
#define PRODUCTION_KD         0.003  // Derivative gain - minimal for noise rejection

// Soft-start ramping parameters
#define SOFT_START_DURATION_MS  2000  // 2 seconds ramp up time
#define SOFT_START_STEPS       20     // Number of ramp steps

#endif // CONFIG_H
