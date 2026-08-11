/**
 * ATtiny85 BLDC Motor PID Controller - v3 (Failsafe)
 *
 * v3 follows the EXACT same tab structure and code flow as v1/v2: same
 * config.h / config_common.h / pid_common.h / rpm_common.h / isr_common.h
 * headers, same setup()/loop() shape, same emergency-stop, soft-start and
 * float PID via computePID_float().
 *
 * Differences vs v2:
 *   - Stall / sensor-loss failsafe: once the motor has reached near-target
 *     speed (valid feedback), if pulses stop for STALL_TIMEOUT_MS the power
 *     is cut to 0 and the controller waits for the motor to be moving again
 *     before re-arming via soft-start. This protects against a stalled motor
 *     or a dead hall sensor without interfering with startup.
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

// --- Global Variables (same as v1/v2) ---
volatile unsigned long pulseInterval = 0;
volatile unsigned long lastPulseMicros = 0;
unsigned long lastRPMCalcTime = 0;

float currentRPM = 0.0;
const float targetRPM = DEFAULT_TARGET_RPM * RPM_CAL_SCALE;

// PID State
float integral = 0.0;
float previousError = 0.0;
float pidOutput = 0.0;

// Filter State (Median + EMA)
float rpmFiltered = 0.0;
float rpmMedianBuffer[MEDIAN_SIZE] = {0};
int rpmMedianIndex = 0;
#define EMA_ALPHA 0.25

// Soft Start State
unsigned long softStartStartTime = 0;
bool softStarting = true;

// Emergency Stop State
unsigned long emergencyStartTime = 0;
bool emergencyRecoveryMode = false;

// v3 failsafe state
bool stalled = false;
bool hasValidFeedback = false;

// Function Prototypes
int applySoftStart(int targetPWM);
float readSensitivityScale();

// Pin Change Interrupt Service Routine for PB3
// Minimum-pulse-width filter rejects glitches / double-pulses.
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
    GIMSK |= (1 << PCIE);
    PCMSK |= (1 << PCINT3);

    // 3. Initialize Output
    analogWrite(PWM_OUTPUT_PIN, PWM_MIN_VALUE);
    delay(1000); // Startup delay
}

void loop() {
    unsigned long currentTime = millis();

    // --- 1. Calculate RPM ---
    if (currentTime - lastRPMCalcTime >= RPM_CALC_INTERVAL) {

        noInterrupts();
        unsigned long interval = pulseInterval;
        interrupts();

        float rawRPM = 0.0;
        if (micros() - lastPulseMicros > RPM_TIMEOUT_US) {
            rawRPM = 0.0;
        } else if (interval > 0) {
            rawRPM = 60000000.0 / interval / DEFAULT_PULSES_PER_REV;
        }

        float medianRPM = getMedian(rawRPM, rpmMedianBuffer, rpmMedianIndex);

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

        // --- v3 failsafe state machine ---
        // Mark feedback valid once we've genuinely reached near-target speed.
        if (currentRPM >= targetRPM * 0.7 && currentRPM <= targetRPM * 1.3) {
            hasValidFeedback = true;
        }

        if (stalled) {
            // Waiting for the motor to be moving again near speed.
            if (currentRPM >= targetRPM * 0.7 && currentRPM <= targetRPM * 1.3) {
                stalled = false;
                softStarting = true; // re-ramp, no hard kick
                softStartStartTime = 0;
                integral = 0.0;
            } else {
                analogWrite(PWM_OUTPUT_PIN, PWM_MIN_VALUE);
                return;
            }
        }

        // Stall detection: valid feedback, then pulses lost for too long.
        if (hasValidFeedback && !stalled &&
            (micros() - lastPulseMicros > STALL_TIMEOUT_MS * 1000UL)) {
            stalled = true;
            integral = 0.0;
        }

        bool trimActive = (digitalRead(POT_ENABLE_PIN) == LOW);
        float sens = trimActive ? readSensitivityScale() : 1.0f;
        float kp = DEFAULT_KP * sens;
        float ki = DEFAULT_KI * sens;
        float kd = DEFAULT_KD * sens;

        float error = targetRPM - currentRPM;

        // --- Emergency Stop Logic ---
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
        if (emergencyRecoveryMode && emergencyStartTime == 0) {
            emergencyRecoveryMode = false;
        }

        // --- Output ---
        analogWrite(PWM_OUTPUT_PIN, pwmValue);
    }
}

// Boosted Soft-Start Implementation (same as v1/v2)
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
