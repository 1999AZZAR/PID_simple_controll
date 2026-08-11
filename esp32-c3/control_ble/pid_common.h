#ifndef PID_COMMON_H
#define PID_COMMON_H

#include <Arduino.h>

#ifndef D_TERM_FILTER_ALPHA
#define D_TERM_FILTER_ALPHA 0.25f
#endif

inline float computePID_float(float error, float &integral, float &previousError,
                              float &filteredDerivative, float kp, float ki, float kd,
                              float integral_min, float integral_max, float output_min,
                              float output_max) {
    float proportional = kp * error;

    float integral_increment = ki * error;
    integral += integral_increment;
    integral = constrain(integral, integral_min, integral_max);

    float rawDerivative = error - previousError;
    previousError = error;
    filteredDerivative =
        D_TERM_FILTER_ALPHA * rawDerivative + (1.0f - D_TERM_FILTER_ALPHA) * filteredDerivative;
    float derivative = kd * filteredDerivative;

    float output = proportional + integral + derivative;
    output = constrain(output, output_min, output_max);

    return output;
}

#endif // PID_COMMON_H
