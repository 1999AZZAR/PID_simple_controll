/**
 * ATtiny85 BLDC Motor PID Controller - v2
 *
 * v2 follows the EXACT same tab structure and code flow as v1 (which reliably
 * drives the motor): same config.h / config_common.h / pid_common.h /
 * rpm_common.h / isr_common.h headers, same setup()/loop() shape, same
 * emergency-stop and soft-start logic, same float PID via computePID_float().
 *
 * Differences vs v1:
 *   - Default clock is 8 MHz (0xE2) instead of v1's 16 MHz.
 *   - The RPM ISR adds a minimum-pulse-width filter (MIN_PULSE_WIDTH_US)
 *     so double-pulses / glitches from the hall sensor are ignored.
 *
 * Hardware Connections:
 * Pin 2 (PB3) -> RPM Sensor (Hall)
 * Pin 5 (PB0) -> PWM Output (ESC / motor driver)
 */

#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#include "config.h"
#include "isr_common.h"
#include "pid_common.h"
#include "rpm_common.h"

#include <avr/interrupt.h>
#include <avr/io.h>

// --- Global Variables (same as v1) ---
volatile unsigned long pulseInterval = 0;
volatile unsigned long lastPulseMicros = 0;
unsigned long lastRPMCalcTime = 0;

float currentRPM = 0.0;
const float targetRPM = DEFAULT_TARGET_RPM * RPM_CAL_SCALE;

// PID State
float integral = 0.0;
float previousError = 0.0;
float pidOutput = 0.0;

// Filter State (Median + EMA) - same as v1
float rpmFiltered = 0.0;
float rpmMedianBuffer[MEDIAN_SIZE] = {0};
int rpmMedianIndex = 0;
#define EMA_ALPHA 0.25

// Soft Start State
unsigned long softStartStartTime = 0;
bool softStarting = true;

// Emergency Stop State (same as v1)
unsigned long emergencyStartTime = 0;
bool emergencyRecoveryMode = false;

// Function Prototypes
int applySoftStart(int targetPWM);
float readSensitivityScale();

// Pin Change Interrupt Service Routine for PB3
// Same as v1 but with a minimum-pulse-width filter to reject glitches
// and double-pulses from the hall sensor.
ISR(PCINT0_vect) {
    // Check for RISING edge (LOW -> HIGH)
    if (digitalRead(RPM_SENSOR_PIN) == HIGH) {
        unsigned long currentMicros = micros();
        unsigned long interval = currentMicros - lastPulseMicros;
        if (interval > MIN_PULSE_WIDTH_US) {
            rpmSensorISR_common(currentMicros, lastPulseMicros, pulseInterval, MIN_PULSE_WIDTH_US);
        }
    }
}

void setup() {
    // 1. Configure Pins
    pinMode(PWM_OUTPUT_PIN, OUTPUT);
    pinMode(RPM_SENSOR_PIN, INPUT_PULLUP);
    pinMode(POT_ENABLE_PIN, INPUT_PULLUP);

    // 2. Enable Pin Change Interrupt on PB3
    GIMSK |= (1 << PCIE);   // Enable Pin Change Interrupts
    PCMSK |= (1 << PCINT3); // Enable Interrupt on PB3

    // 3. Initialize Output
    analogWrite(PWM_OUTPUT_PIN, PWM_MIN_VALUE);
    delay(1000); // Startup delay
}

void loop() {
    unsigned long currentTime = millis();

    // --- 1. Calculate RPM ---
    if (currentTime - lastRPMCalcTime >= RPM_CALC_INTERVAL) {

        // Atomic read
        noInterrupts();
        unsigned long interval = pulseInterval;
        interrupts();

        float rawRPM = 0.0;
        // Timeout check
        if (micros() - lastPulseMicros > RPM_TIMEOUT_US) {
            rawRPM = 0.0;
        } else if (interval > 0) {
            rawRPM = 60000000.0 / interval / DEFAULT_PULSES_PER_REV;
        }

        // Apply Median Filter (Spike Rejection)
        float medianRPM = getMedian(rawRPM, rpmMedianBuffer, rpmMedianIndex);

        // Apply EMA Filter (Smoothing)
        if (rpmFiltered == 0.0 && medianRPM > 0.0) {
            rpmFiltered = medianRPM;
            for (int i = 0; i < MEDIAN_SIZE; i++)
                rpmMedianBuffer[i] = medianRPM;
        } else {
            updateEMA(rpmFiltered, medianRPM, EMA_ALPHA);
        }
        currentRPM = rpmFiltered;
        lastRPMCalcTime = currentTime;
    }

    // --- 2. Control Loop ---
    static unsigned long lastControlTime = 0;
    if (currentTime - lastControlTime >= CONTROL_PERIOD_MS) {
        lastControlTime = currentTime;

        bool trimActive = (digitalRead(POT_ENABLE_PIN) == LOW);
        float sens = trimActive ? readSensitivityScale() : 1.0f;
        float kp = DEFAULT_KP * sens;
        float ki = DEFAULT_KI * sens;
        float kd = DEFAULT_KD * sens;

        float error = targetRPM - currentRPM;

        // --- 3. Emergency Stop Logic (same as v1) ---
        int pwmValue;

        if (abs(error) > 2000 && !emergencyRecoveryMode) {
            if (emergencyStartTime == 0) {
                emergencyStartTime = millis();
            }
            pwmValue = PWM_MIN_VALUE;
        } else {
            // Normal PID Operation
            pidOutput =
                computePID_float(error, integral, previousError, kp, ki, kd, INTEGRAL_WINDUP_MIN,
                                 INTEGRAL_WINDUP_MAX, PID_OUTPUT_MIN, PID_OUTPUT_MAX);

            pwmValue = map(pidOutput, PID_OUTPUT_MIN, PID_OUTPUT_MAX, PWM_MIN_VALUE, PWM_MAX_VALUE);
            pwmValue = constrain(pwmValue, PWM_MIN_THRESHOLD, PWM_MAX_VALUE);

            // Apply Soft Start
            pwmValue = applySoftStart(pwmValue);
        }

        // Reset emergency timer after 2 seconds
        if (emergencyStartTime > 0 && millis() - emergencyStartTime > 2000) {
            emergencyStartTime = 0;
            emergencyRecoveryMode = true;
        }
        // Clear recovery mode
        if (emergencyRecoveryMode && emergencyStartTime == 0) {
            emergencyRecoveryMode = false;
        }

        // --- 4. Output ---
        analogWrite(PWM_OUTPUT_PIN, pwmValue);
    }
}

// Boosted Soft-Start Implementation (same as v1)
int applySoftStart(int targetPWM) {
    if (!softStarting)
        return targetPWM;

    if (softStartStartTime == 0)
        softStartStartTime = millis();

    unsigned long elapsed = millis() - softStartStartTime;
    if (elapsed >= SOFT_START_DURATION_MS) {
        softStarting = false;
        return targetPWM;
    }

    float progress = (float)elapsed / SOFT_START_DURATION_MS;

    // Kickstart from PWM_MIN_THRESHOLD (usually 45)
    int kickstartPWM = PWM_MIN_THRESHOLD + (int)((targetPWM - PWM_MIN_THRESHOLD) * progress);

    if (kickstartPWM > targetPWM)
        return targetPWM;
    return kickstartPWM;
}

float readSensitivityScale() {
    int raw = analogRead(POT_SENSITIVITY_PIN);
    return PID_SENSITIVITY_MIN +
           ((float)raw / 1023.0f) * (PID_SENSITIVITY_MAX - PID_SENSITIVITY_MIN);
}
