#ifndef CONFIG_ATTINY_H
#define CONFIG_ATTINY_H

// ATtiny85 configuration
#define F_CPU 8000000UL  // 8MHz internal oscillator

// Pin definitions (ATtiny85 Arduino-style pin numbering)
#define RPM_SENSOR_PIN     3    // Arduino pin 3 (PB3), BLDC Hall sensor input (interrupt capable)
#define PWM_OUTPUT_PIN     0    // Arduino pin 0 (PB0), PWM capable

// Control parameters
#define CONTROL_LOOP_HZ     100 // Control loop frequency (100 Hz)
#define CONTROL_PERIOD_MS   (1000 / CONTROL_LOOP_HZ)

// RPM calculation parameters
#define PULSES_PER_REV      18   // Number of pulses per revolution
#define RPM_CALC_INTERVAL   100 // RPM calculation interval in ms
#define MIN_PULSE_WIDTH_US  100 // Minimum pulse width to reject EMI spikes (100-500us)

// PID limits
#define PID_OUTPUT_MIN      -255 // Minimum PID output
#define PID_OUTPUT_MAX      255  // Maximum PID output
#define INTEGRAL_WINDUP_MIN -50  // Reduced anti-windup for ATtiny85
#define INTEGRAL_WINDUP_MAX 50   // Reduced anti-windup for ATtiny85

// Production mode default values (pre-tuned from Arduino)
#define PRODUCTION_TARGET_RPM 1440.0
#define PRODUCTION_KP         0.5
#define PRODUCTION_KI         0.1
#define PRODUCTION_KD         0.01

// Safety parameters
// Safety features removed for minimal size optimization

// Soft-start ramping parameters
#define SOFT_START_DURATION_MS  2000  // 2 seconds ramp up time

#endif // CONFIG_ATTINY_H
