#ifndef CONFIG_ATTINY_H
#define CONFIG_ATTINY_H

// ATtiny85 configuration
#define F_CPU 8000000UL  // 8MHz internal oscillator

// Pin definitions (ATtiny85 Arduino-style pin numbering)
#define RPM_SENSOR_PIN     3    // Arduino pin 3 (PB3), BLDC Hall sensor input (interrupt capable)
#define PWM_OUTPUT_PIN     0    // Arduino pin 0 (PB0), PWM capable

// Control parameters - Optimized for ATtiny85 real-time performance
#define CONTROL_LOOP_HZ     80  // Control loop frequency (80 Hz) - balanced for ATtiny85 capabilities
#define CONTROL_PERIOD_MS   (1000 / CONTROL_LOOP_HZ)

// RPM calculation parameters
#define PULSES_PER_REV      18   // Number of pulses per revolution
#define RPM_CALC_INTERVAL   50  // RPM calculation interval (50ms) - optimized for ATtiny85 resources
#define MIN_PULSE_WIDTH_US  75  // Minimum pulse width (75μs) - balanced EMI rejection vs responsiveness

// PID limits
#define PID_OUTPUT_MIN      -255 // Minimum PID output
#define PID_OUTPUT_MAX      255  // Maximum PID output
#define INTEGRAL_WINDUP_MIN -50  // Reduced anti-windup for ATtiny85
#define INTEGRAL_WINDUP_MAX 50   // Reduced anti-windup for ATtiny85

// Production mode default values - Optimized for 80Hz ATtiny85 control loop
#define PRODUCTION_TARGET_RPM 1440.0
#define PRODUCTION_KP         0.35   // Proportional gain - optimized for ATtiny85 80Hz loop
#define PRODUCTION_KI         0.025  // Integral gain - conservative for integer math
#define PRODUCTION_KD         0.004  // Derivative gain - minimal noise filtering

// Safety parameters
// Safety features removed for minimal size optimization

// Soft-start ramping parameters
#define SOFT_START_DURATION_MS  2000  // 2 seconds ramp up time

#endif // CONFIG_ATTINY_H
