#ifndef CONFIG_COMMON_H
#define CONFIG_COMMON_H

// Shared configuration constants used across all BLDC motor controller projects

// RPM calculation parameters (shared)
#define RPM_TIMEOUT_US 500000 // Timeout for RPM detection (500ms)
#define MIN_PULSE_WIDTH_US 50 // Minimum pulse width (50μs)
#define RPM_CALC_INTERVAL 10  // RPM calculation interval (10ms)
#define RPM_FILTER_SIZE 5     // Moving average filter size (Matches Arduino Uno)

// PID control parameters (shared)
#define PID_OUTPUT_MIN -5000      // Minimum PID output (scaled)
#define PID_OUTPUT_MAX 5000       // Maximum PID output (scaled)
#define INTEGRAL_WINDUP_MIN -5000 // Anti-windup integral minimum (Matches Output Min)
#define INTEGRAL_WINDUP_MAX 5000  // Anti-windup integral maximum (Matches Output Max)

// PWM output parameters (shared)
#define PWM_MIN_VALUE 0      // Minimum PWM value (0 = motor stopped)
#define PWM_MAX_VALUE 255    // Maximum PWM value (255 = full speed)
#define PWM_MIN_THRESHOLD 45 // Minimum PWM threshold for motor torque (~18% of full range)

// Default PID parameters (Matches Stable Arduino Uno Version)
#define DEFAULT_TARGET_RPM 1440.0 // Default target RPM
#define DEFAULT_KP 0.150          // Default proportional gain
#define DEFAULT_KI 0.080          // Default integral gain
#define DEFAULT_KD 0.015          // Default derivative gain

#endif // CONFIG_COMMON_H
