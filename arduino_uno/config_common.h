#ifndef CONFIG_COMMON_H
#define CONFIG_COMMON_H

// Shared configuration constants used across all BLDC motor controller projects

// RPM calculation parameters (shared)
#define RPM_TIMEOUT_US      500000  // Timeout for RPM detection (500ms)
#define MIN_PULSE_WIDTH_US  50      // Minimum pulse width (50μs)
#define RPM_CALC_INTERVAL   10      // RPM calculation interval (10ms)
#define RPM_FILTER_SIZE     5       // Moving average filter size (increased for stability)

// PID control parameters (shared)
#define PID_OUTPUT_MIN      -5000   // Minimum PID output (increased range for better resolution)
#define PID_OUTPUT_MAX      5000    // Maximum PID output (increased range for better resolution)
#define INTEGRAL_WINDUP_MIN -1000   // Anti-windup integral minimum (increased for more integral accumulation)
#define INTEGRAL_WINDUP_MAX 1000    // Anti-windup integral maximum (increased for more integral accumulation)

// PWM output parameters (shared)
#define PWM_MIN_VALUE       0       // Minimum PWM value (0 = motor stopped)
#define PWM_MAX_VALUE       255     // Maximum PWM value (255 = full speed)
#define PWM_MIN_THRESHOLD   20      // Minimum PWM threshold for motor torque (~8% of full range)

// Control loop timing (shared)
#define CONTROL_LOOP_HZ     200     // Control loop frequency (200 Hz)
#define CONTROL_PERIOD_MS   (1000 / CONTROL_LOOP_HZ)

// Default PID parameters (can be overridden by each project)
#define DEFAULT_TARGET_RPM 1440.0   // Default target RPM
#define DEFAULT_KP         0.150    // Default proportional gain (reduced for stability)
#define DEFAULT_KI         0.080    // Default integral gain (reduced for stability)
#define DEFAULT_KD         0.015    // Default derivative gain (reduced for stability)

// Spike dampening and smoothing parameters
#define DERIVATIVE_FILTER_ALPHA 0.2   // Derivative low-pass filter coefficient (0.1=smooth, 0.5=responsive)
#define OUTPUT_SLEW_RATE        80.0  // Max PID output change per iteration (limits rate of change)
#define SETPOINT_RAMP_RATE      50.0  // RPM per iteration for setpoint ramping during startup

// Setpoint ramping configuration
#define SETPOINT_RAMP_ENABLED   true  // Enable/disable setpoint ramping
#define SETPOINT_RAMP_THRESHOLD 100.0 // Only ramp when error exceeds this value (RPM)

#endif // CONFIG_COMMON_H
