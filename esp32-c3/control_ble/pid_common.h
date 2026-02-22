#ifndef PID_COMMON_H
#define PID_COMMON_H

#include <Arduino.h>

inline float computePID_float(float error, float& integral, float& previousError,
                              float kp, float ki, float kd,
                              float integral_min, float integral_max,
                              float output_min, float output_max) {
    // Proportional
    float proportional = kp * error;

    // Integral
    float integral_increment = ki * error;
    integral += integral_increment;
    integral = constrain(integral, integral_min, integral_max);

    // Derivative
    float derivative = kd * (error - previousError);
    previousError = error;

    // Total Output
    float output = proportional + integral + derivative;
    output = constrain(output, output_min, output_max);

    return output;
}

#endif // PID_COMMON_H
