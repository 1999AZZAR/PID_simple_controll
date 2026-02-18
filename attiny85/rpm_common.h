#ifndef RPM_COMMON_H
#define RPM_COMMON_H

#include <Arduino.h>

// Shared RPM calculation functions with period measurement

// Moving average filter for RPM smoothing
#define RPM_FILTER_SIZE 5

// Timer rollover safe interval calculation
// Returns the time difference between two timestamps, handling rollover
inline unsigned long safeInterval(unsigned long current, unsigned long previous) {
    if (current >= previous) {
        // Normal case: no rollover
        return current - previous;
    } else {
        // Rollover occurred: timer wrapped from 0xFFFFFFFF to 0
        return (0xFFFFFFFFUL - previous) + current + 1;
    }
}

// Moving average filter update
inline void updateMovingAverage(float& filteredValue, float newValue,
                               float history[], int& historyIndex, int filterSize) {
    // Add new value to history
    history[historyIndex] = newValue;
    historyIndex = (historyIndex + 1) % filterSize;

    // Calculate filtered average
    float sum = 0.0;
    for(int i = 0; i < filterSize; i++) {
        sum += history[i];
    }
    filteredValue = sum / filterSize;
}

// Moving average filter update for integer values
inline void updateMovingAverageInt(int& filteredValue, int newValue,
                                  int history[], int& historyIndex, int filterSize) {
    // Add new value to history
    history[historyIndex] = newValue;
    historyIndex = (historyIndex + 1) % filterSize;

    // Calculate filtered average
    long sum = 0;
    for(int i = 0; i < filterSize; i++) {
        sum += history[i];
    }
    filteredValue = (int)(sum / filterSize);
}

// Exponential Moving Average (EMA) filter update for integer values
// alpha_scaled: Smoothing factor scaled by 100 (e.g. 25 = 0.25)
inline void updateEMAInt(int& filteredValue, int newValue, int alpha_scaled) {
    // filtered = (alpha * new) + ((100 - alpha) * old) / 100
    filteredValue = (int)(((long)alpha_scaled * newValue + (long)(100 - alpha_scaled) * filteredValue) / 100);
}

#endif // RPM_COMMON_H
