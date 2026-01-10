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
#define DEFAULT_KP         0.900    // Default proportional gain
#define DEFAULT_KI         1.000    // Default integral gain
#define DEFAULT_KD         0.0562   // Default derivative gain

#endif // CONFIG_COMMON_H
