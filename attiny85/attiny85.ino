/**
 * BLDC Motor PID Controller - ATtiny85 Production Version (Refined)
 *
 * Rewritten to behave EXACTLY like the Arduino Uno version:
 * 1. Uses Floating Point Math (same PID precision)
 * 2. Uses Standard Arduino Timing (millis/micros)
 * 3. Uses Standard analogWrite (PWM)
 * 4. Correct PCINT Interrupt for PB3
 *
 * Hardware Connections:
 * Pin 2 (PB3) -> RPM Sensor (Hall)
 * Pin 5 (PB0) -> PWM Output (ESC)
 */

#include "config.h"
#include "pid_common.h"
#include "rpm_common.h"
#include "isr_common.h"

#include <avr/io.h>
#include <avr/interrupt.h>

// Global Variables (Float - Matching Arduino Uno)
volatile unsigned long pulseInterval = 0;
volatile unsigned long lastPulseMicros = 0;
unsigned long lastRPMCalcTime = 0;

float currentRPM = 0.0;
float targetRPM = DEFAULT_TARGET_RPM; // 1440.0

// PID State (Float)
float integral = 0.0;
float previousError = 0.0;
float pidOutput = 0.0;

// Filter State (Median + EMA)
// Double filtering for maximum stability on noisy internal clock
float rpmFiltered = 0.0;
float rpmMedianBuffer[MEDIAN_SIZE] = {0}; // Buffer for median filter
int rpmMedianIndex = 0;
#define EMA_ALPHA 0.40 // Aggressive alpha (0.40) for fast response to match Arduino speed

// Soft Start State
unsigned long softStartStartTime = 0;
bool softStarting = true;

// Pin Change Interrupt Service Routine for PB3
// Fires on BOTH Rising and Falling edges
ISR(PCINT0_vect) {
    // We only care about RISING edge (LOW -> HIGH)
    if (digitalRead(RPM_SENSOR_PIN) == HIGH) { 
        unsigned long currentMicros = micros();
        rpmSensorISR_common(currentMicros, lastPulseMicros, pulseInterval, MIN_PULSE_WIDTH_US);
    }
}

void setup() {
    // 1. Configure Pins
    pinMode(PWM_OUTPUT_PIN, OUTPUT);
    pinMode(RPM_SENSOR_PIN, INPUT_PULLUP);

    // 2. Enable Pin Change Interrupt on PB3 (Pin 2)
    GIMSK |= (1 << PCIE);   // Enable Pin Change Interrupts
    PCMSK |= (1 << PCINT3); // Enable Interrupt on PB3

    // 3. Initialize Output
    analogWrite(PWM_OUTPUT_PIN, 0);
    delay(1000); // Startup delay
}

void loop() {
    unsigned long currentTime = millis();

    // Calculate RPM at 10ms interval (same as Arduino)
    if (currentTime - lastRPMCalcTime >= RPM_CALC_INTERVAL) {
        
        // 1. Read Interval Atomically
        noInterrupts();
        unsigned long interval = pulseInterval;
        interrupts();

        // 2. Calculate Raw RPM
        float rawRPM = 0.0;
        // Check for timeout (motor stopped)
        if (micros() - lastPulseMicros > RPM_TIMEOUT_US) {
            rawRPM = 0.0;
        } else if (interval > 0) {
            // RPM = 60,000,000 / (interval * pulsesPerRev)
            rawRPM = 60000000.0 / interval / DEFAULT_PULSES_PER_REV;
        }

        // 3. Apply Median + EMA Filter
        // First, remove spikes with Median Filter
        float medianRPM = getMedian(rawRPM, rpmMedianBuffer, rpmMedianIndex);
        
        // Then, smooth the clean signal with EMA
        if (rpmFiltered == 0.0 && medianRPM > 0.0) {
            rpmFiltered = medianRPM; // Instant start for first valid reading
            // Fill buffer to prevent drag
            for(int i=0; i<MEDIAN_SIZE; i++) rpmMedianBuffer[i] = medianRPM;
        } else {
            updateEMA(rpmFiltered, medianRPM, EMA_ALPHA);
        }
        currentRPM = rpmFiltered;
        
        lastRPMCalcTime = currentTime;
    }

    // Control Loop (200Hz)
    static unsigned long lastControlTime = 0;
    if (currentTime - lastControlTime >= CONTROL_PERIOD_MS) {
        lastControlTime = currentTime;

        // 4. Compute PID (Float)
        float error = targetRPM - currentRPM;
        
        // Using common float PID function (same as Arduino)
        pidOutput = computePID_float(error, integral, previousError,
                                   DEFAULT_KP, DEFAULT_KI, DEFAULT_KD,
                                   INTEGRAL_WINDUP_MIN, INTEGRAL_WINDUP_MAX,
                                   PID_OUTPUT_MIN, PID_OUTPUT_MAX);

        // 5. Map to PWM
        // Map +/- 5000 PID output to 0-255 PWM
        int pwmValue = map(pidOutput, PID_OUTPUT_MIN, PID_OUTPUT_MAX, PWM_MIN_VALUE, PWM_MAX_VALUE);
        pwmValue = constrain(pwmValue, PWM_MIN_THRESHOLD, PWM_MAX_VALUE);

        // 6. Apply Soft Start
        pwmValue = applySoftStart(pwmValue);

        // 7. Output
        analogWrite(PWM_OUTPUT_PIN, pwmValue);
    }
}

// Boosted Soft-Start Implementation
int applySoftStart(int targetPWM) {
    if (!softStarting) return targetPWM;

    if (softStartStartTime == 0) softStartStartTime = millis();

    unsigned long elapsed = millis() - softStartStartTime;
    if (elapsed >= SOFT_START_DURATION_MS) {
        softStarting = false;
        return targetPWM;
    }

    float progress = (float)elapsed / SOFT_START_DURATION_MS;
    
    // Kickstart from PWM_MIN_THRESHOLD (45) to overcome friction
    int kickstartPWM = PWM_MIN_THRESHOLD + (int)((targetPWM - PWM_MIN_THRESHOLD) * progress);
    
    if (kickstartPWM > targetPWM) return targetPWM;
    return kickstartPWM;
}
