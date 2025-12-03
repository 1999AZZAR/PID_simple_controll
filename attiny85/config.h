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
#define DEFAULT_PULSES_PER_REV 24   // Default number of pulses per revolution got from 4 cycles×6 steps=24 state changes per revolution.
#define RPM_CALC_INTERVAL   25  // RPM calculation interval (25ms) - faster updates for smooth control
#define MIN_PULSE_WIDTH_US  100  // Minimum pulse width (100μs) - reduced for better responsiveness

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

// Safety parameters
// Safety features removed for minimal size optimization

// Soft-start ramping parameters
#define SOFT_START_DURATION_MS  2000  // 2 seconds ramp up time

#endif // CONFIG_ATTINY_H
