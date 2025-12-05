#ifndef CONFIG_COMMON_H
#define CONFIG_COMMON_H

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

// Control loop timing (shared)
#define CONTROL_LOOP_HZ     200     // Control loop frequency (200 Hz)
#define CONTROL_PERIOD_MS   (1000 / CONTROL_LOOP_HZ)

// Default PID parameters (can be overridden by each project)
#define DEFAULT_TARGET_RPM 1440.0   // Default target RPM
#define DEFAULT_KP         0.8      // Default proportional gain (further reduced for stability)
#define DEFAULT_KI         0.008    // Default integral gain (further reduced for accuracy)
#define DEFAULT_KD         0.005    // Default derivative gain (further increased for damping)

#endif // CONFIG_COMMON_H
