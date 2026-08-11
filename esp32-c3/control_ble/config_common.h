#ifndef CONFIG_COMMON_H
#define CONFIG_COMMON_H

// Control Loop
#define CONTROL_LOOP_HZ     200
#define CONTROL_PERIOD_MS   (1000 / CONTROL_LOOP_HZ)

// PID Constants (Proven Values)
#define DEFAULT_TARGET_RPM 1440.0
#define DEFAULT_KP         0.150
#define DEFAULT_KI         0.080
#define DEFAULT_KD         0.015

// Limits
#define PID_OUTPUT_MIN      -5000
#define PID_OUTPUT_MAX      5000
#define INTEGRAL_WINDUP_MIN -1000
#define INTEGRAL_WINDUP_MAX 1000

// PWM Limits (8-bit resolution 0-255 for compatibility logic)
#define PWM_MIN_VALUE       0
#define PWM_MAX_VALUE       255
#define PWM_MIN_THRESHOLD   45  // Kickstart torque
#define PWM_IDLE            0   // Safe output when stalled (motor off)

// Target RPM Limits (safety)
#define TARGET_RPM_MIN 100.0
#define TARGET_RPM_MAX 10000.0

// Stall / sensor-loss failsafe: cut power if no pulse for this long while the
// motor has reached near-target speed. 150ms is ~14x the target pulse period.
#define STALL_TIMEOUT_MS 150

#endif // CONFIG_COMMON_H
