#ifndef CONFIG_COMMON_H
#define CONFIG_COMMON_H

// ATTiny85 Configuration Selector
// Uncomment ONE of the following lines to choose your configuration:

#define USE_EXTERNAL_CRYSTAL  // Use 20MHz external crystal (requires hardware)
//#define USE_INTERNAL_OSCILLATOR  // Use 8MHz internal oscillator (default)

// Include the appropriate configuration based on selection
#if defined(USE_EXTERNAL_CRYSTAL)
#include "config_external.h"
#elif defined(USE_INTERNAL_OSCILLATOR)
#include "config_internal.h"
#else
// Default to internal oscillator if nothing is selected
#warning "No oscillator type selected, defaulting to internal oscillator"
#include "config_internal.h"
#endif

// Shared configuration constants used across all BLDC motor controller projects

// RPM calculation parameters (shared)
#define RPM_TIMEOUT_US      500000  // Timeout for RPM detection (500ms)
#define MIN_PULSE_WIDTH_US  50      // Minimum pulse width (50μs)
#define RPM_CALC_INTERVAL   10      // RPM calculation interval (10ms)
#define RPM_FILTER_SIZE     3       // Moving average filter size (reduced for accuracy)

// PID control parameters (shared)
#define PID_OUTPUT_MIN      -1000   // Minimum PID output
#define PID_OUTPUT_MAX      1000    // Maximum PID output
#define INTEGRAL_WINDUP_MIN -200    // Anti-windup integral minimum
#define INTEGRAL_WINDUP_MAX 200     // Anti-windup integral maximum

// Default PID parameters (can be overridden by each project)
#define DEFAULT_TARGET_RPM 1440.0   // Default target RPM
#define DEFAULT_KP         0.150    // Default proportional gain (matched with Arduino Uno)
#define DEFAULT_KI         0.080    // Default integral gain
#define DEFAULT_KD         0.015    // Default derivative gain

// Spike dampening parameters (scaled for integer math)
// DERIVATIVE_FILTER_ALPHA: 0.2 = 20/100, stored as percentage for integer math
#define DERIVATIVE_FILTER_ALPHA_SCALED 20   // 20% = 0.2 (range 10-50)
#define OUTPUT_SLEW_RATE_SCALED        8    // Max output change per iteration (scaled)
#define SETPOINT_RAMP_RATE_SCALED      50   // RPM*10 per iteration (5.0 RPM actual)
#define SETPOINT_RAMP_ENABLED          1    // 1 = enabled, 0 = disabled

// Emergency handler parameters (scaled for integer math)
#define EMERGENCY_ERROR_THRESHOLD_SCALED 20000  // Error * 10 (2000 RPM actual)
#define EMERGENCY_RAMPDOWN_RATE          5      // PWM reduction per iteration
#define EMERGENCY_MIN_PWM                30     // Minimum PWM during emergency
#define EMERGENCY_RECOVERY_TIME_MS       3000   // Recovery wait time (ms)
#define EMERGENCY_FULL_STOP              0      // 0 = stop at MIN_PWM, 1 = stop at 0

#endif // CONFIG_COMMON_H
