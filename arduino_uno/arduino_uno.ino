/**
 * BLDC Motor PID Controller - Arduino Uno Version
 *
 * This Arduino sketch implements a PID controller to maintain a BLDC motor
 * at exactly 1440 RPM, even under varying load conditions.
 *
 * Motor Compatibility:
 * - Designed for 3-Hall BLDC motors (such as 42BLF20-22.0223)
 * - Works with any BLDC motor that has 3 built-in Hall effect sensors
 * - Hall sensors provide 6 pulses per electrical revolution
 * - Compatible with standard BLDC motor controllers (ESC)
 *
 * Features:
 * - PID control with anti-windup protection
 * - RPM feedback via Hall sensor (direct motor connection)
 * - PWM output to ESC
 * - Optional sensitivity trim: D3 + A4 scale Kp/Ki/Kd together (target RPM fixed at
 * DEFAULT_TARGET_RPM)
 * - Serial Plotter output for monitoring
 *
 * Hardware Requirements:
 * - Arduino board (Uno, Mega, or similar)
 * - 3-Hall BLDC motor (such as 42BLF20-22.0223 or equivalent)
 * - BLDC motor controller (ESC) compatible with the motor
 * - Any one Hall sensor wire from the BLDC motor (Hall A, B, or C)
 * - Optional: jumper on D3 + trim pot on A4 for PID gain scale
 *
 * Configuration:
 * - All configuration parameters are defined in config.h
 * - Modify config.h to change default values, pin assignments, etc.
 *
 * Author: azzar budiyanto
 * Co-Author: azzar persona (AI assistant)
 * Date: January 2026
 */

// Include configuration header (contains all pin definitions, constants, and settings)
#include "config.h"

// Include shared common headers
#include "isr_common.h"
#include "pid_common.h"
#include "rpm_common.h"

// Serial removed - no interactive commands

// Global variables
volatile unsigned long pulseInterval = 0; // Time interval between pulses in microseconds
volatile unsigned long lastPulseMicros = 0;
// Safety features removed for simplified operation
unsigned long lastRPMCalcTime = 0;
float currentRPM = 0.0;
const float targetRPM = DEFAULT_TARGET_RPM;
const int pulsesPerRev = DEFAULT_PULSES_PER_REV;
float sensitivityScale = 1.0f;
int lastPWMValue = 0;

// Exponential Moving Average (EMA) filter for RPM smoothing
#define EMA_ALPHA 0.25 // Smoothing factor (0.1 = stable, 1.0 = instant)
float rpmFiltered = 0.0;

// PID variables
float kp = DEFAULT_KP;
float ki = DEFAULT_KI;
float kd = DEFAULT_KD;
float previousError = 0.0;
float integral = 0.0;
float pidOutput = 0.0;

// Safety features

// Soft-start ramping to avoid current surges
unsigned long softStartStartTime = 0;
bool softStarting = true;
int softStartStep = 0;

// Function prototypes
void rpmSensorISR();
float calculateRPM();
float computePID(float error);
void outputToESC(int pwmValue);
void printToSerialPlotter();
float readSensitivityScale();

void setup() {
    // Initialize serial communication
    Serial.begin(115200);

    // Configure pins
    pinMode(RPM_SENSOR_PIN, INPUT_PULLUP);
    pinMode(PWM_OUTPUT_PIN, OUTPUT);
    pinMode(POT_ENABLE_PIN, INPUT_PULLUP);

    // Set PWM frequency to 50Hz (standard for RC ESCs)
    // Timer1 is used for PWM on pin 9 (OC1A)
    TCCR1B = (TCCR1B & 0xF8) | 0x03; // Set prescaler to 64 for ~50Hz

    // Attach interrupt for BLDC Hall sensor
    attachInterrupt(digitalPinToInterrupt(RPM_SENSOR_PIN), rpmSensorISR, RISING);

    // Initialize PWM output to stopped position
    analogWrite(PWM_OUTPUT_PIN, PWM_MIN_VALUE);

    // Brief startup delay
    delay(1000);

    // EEPROM loading removed

    Serial.println(F("BLDC PID Controller Started"));
    Serial.println(F("Mode: Production (default)"));
    Serial.print(F("Target RPM: "));
    Serial.println(DEFAULT_TARGET_RPM);
}

void loop() {

    unsigned long currentTime = millis();

    bool trimActive = (digitalRead(POT_ENABLE_PIN) == LOW);
    sensitivityScale = trimActive ? readSensitivityScale() : 1.0f;
    kp = DEFAULT_KP * sensitivityScale;
    ki = DEFAULT_KI * sensitivityScale;
    kd = DEFAULT_KD * sensitivityScale;

    static bool lastTrim = !trimActive;
    if (trimActive != lastTrim) {
        Serial.print(F("Mode: "));
        Serial.println(trimActive ? F("Sensitivity trim") : F("Production"));
        lastTrim = trimActive;
    }

    // Calculate RPM at regular intervals
    if (currentTime - lastRPMCalcTime >= RPM_CALC_INTERVAL) {
        currentRPM = calculateRPM();
        lastRPMCalcTime = currentTime;
    }

    // Safety features removed for simplified operation

    // Compute PID output
    float error = targetRPM - currentRPM;
    pidOutput = computePID(error);

    // Emergency stop if error is extremely large (motor out of control)
    int pwmValue;
    static unsigned long emergencyStartTime = 0;
    static bool emergencyRecoveryMode = false;

    if (abs(error) > 2000 &&
        !emergencyRecoveryMode) { // If error > 2000 RPM, emergency stop (increased threshold)
        if (emergencyStartTime == 0) {
            emergencyStartTime = millis();
            Serial.println("EMERGENCY STOP: Motor out of control!");
        }
        pwmValue = 0; // Cut power completely for 2 seconds
    } else {
        // Convert PID output to PWM value and output to ESC
        // Map PID output range to PWM range with minimum threshold for torque
        pwmValue = map(pidOutput, PID_OUTPUT_MIN, PID_OUTPUT_MAX, PWM_MIN_VALUE, PWM_MAX_VALUE);
        pwmValue = constrain(pwmValue, PWM_MIN_THRESHOLD,
                             PWM_MAX_VALUE); // Minimum threshold for motor torque
    }

    // Reset emergency timer after 2 seconds regardless of current error state
    if (emergencyStartTime > 0 && millis() - emergencyStartTime > 2000) {
        emergencyStartTime = 0;       // Reset emergency after 2 seconds
        emergencyRecoveryMode = true; // Allow one recovery cycle before checking emergency again
    }

    // Clear recovery mode after one cycle to allow emergency checks again
    if (emergencyRecoveryMode && emergencyStartTime == 0) {
        emergencyRecoveryMode = false;
    }

    // Temporarily disable hysteresis for debugging
    lastPWMValue = pwmValue;
    outputToESC(pwmValue);

    // Output to Serial Plotter for monitoring
    printToSerialPlotter();

    // Control loop timing
    delay(CONTROL_PERIOD_MS);
}

// Interrupt service routine for RPM sensor with debounce filtering
void rpmSensorISR() {
    unsigned long currentMicros = micros();
    rpmSensorISR_common(currentMicros, lastPulseMicros, pulseInterval, MIN_PULSE_WIDTH_US);
    // Safety features removed for simplified operation
}

// Calculate RPM from pulse interval with timeout detection and moving average filtering
float calculateRPM() {
    static unsigned long lastCalcTime = 0;

    unsigned long currentTime = micros(); // Use micros for maximum precision

    if (currentTime - lastCalcTime >= RPM_CALC_INTERVAL * 1000UL) { // Convert ms to microseconds
        float rpm = 0.0;

        // Timeout check: if no pulses received within timeout period, motor is stopped
        if (currentTime - lastPulseMicros > RPM_TIMEOUT_US) {
            rpm = 0.0; // Motor stopped
        } else {
            // Atomic read of volatile pulseInterval to avoid race conditions
            noInterrupts();
            unsigned long interval = pulseInterval;
            interrupts();

            // Calculate RPM using period measurement: RPM = (60,000,000) / (interval_μs *
            // pulses_per_rev) Use sequential division to avoid intermediate multiplication overflow
            if (interval > 0) {
                rpm = 60000000.0 / interval / pulsesPerRev;
            }
        }

        // Apply Exponential Moving Average (EMA) filter
        if (rpmFiltered == 0.0 && rpm > 0.0) {
            rpmFiltered = rpm; // Initialize with first valid reading to avoid startup lag
        } else {
            updateEMA(rpmFiltered, rpm, EMA_ALPHA);
        }

        lastCalcTime = currentTime;

        return rpmFiltered; // Return filtered value for maximum accuracy
    }

    return rpmFiltered; // Return previous filtered value if not enough time has passed
}

float readSensitivityScale() {
    int raw = analogRead(POT_SENSITIVITY_PIN);
    return PID_SENSITIVITY_MIN +
           ((float)raw / 1023.0f) * (PID_SENSITIVITY_MAX - PID_SENSITIVITY_MIN);
}

// Compute PID output using shared function with overflow protection
float computePID(float error) {
    return computePID_float(error, integral, previousError, kp, ki, kd, INTEGRAL_WINDUP_MIN,
                            INTEGRAL_WINDUP_MAX, PID_OUTPUT_MIN, PID_OUTPUT_MAX);
}

// Apply soft-start ramping to avoid current surges
int applySoftStart(int targetPWM) {
    if (!softStarting) {
        return targetPWM; // Normal operation
    }

    unsigned long currentTime = millis();

    if (softStartStartTime == 0) {
        softStartStartTime = currentTime;
    }

    unsigned long elapsed = currentTime - softStartStartTime;
    float rampProgress = (float)elapsed / SOFT_START_DURATION_MS;

    if (rampProgress >= 1.0) {
        // Soft-start complete
        softStarting = false;
        return targetPWM;
    }

    // Apply ramped output
    // Boosted ramp: start at PWM_MIN_THRESHOLD and ramp up to targetPWM
    // Formula: output = min_threshold + (target - min_threshold) * rampProgress
    // This ensures motor overcomes static friction immediately
    int kickstartPWM = PWM_MIN_THRESHOLD + (int)((targetPWM - PWM_MIN_THRESHOLD) * rampProgress);

    // Ensure we don't exceed targetPWM
    if (kickstartPWM > targetPWM) {
        return targetPWM;
    }

    return kickstartPWM;
}

// Output PWM value to ESC with soft-start protection
void outputToESC(int pwmValue) {
    int safePWM = applySoftStart(pwmValue);
    analogWrite(PWM_OUTPUT_PIN, safePWM);
}

// Print data for Serial Plotter
void printToSerialPlotter() {
    Serial.print("Target:");
    Serial.print(targetRPM);
    Serial.print(",");
    Serial.print("Current:");
    Serial.print(currentRPM);
    Serial.print(",");
    Serial.print("Error:");
    Serial.print(targetRPM - currentRPM);
    Serial.print(",");
    Serial.print("PID_Output:");
    Serial.print(pidOutput);
    Serial.print(",");
    Serial.print("PWM:");
    Serial.print(lastPWMValue); // Show actual PWM being sent
    Serial.print(",");
    Serial.print("Kp:");
    Serial.print(kp, 2);
    Serial.print(",");
    Serial.print("Ki:");
    Serial.print(ki, 2);
    Serial.print(",");
    Serial.print("Kd:");
    Serial.print(kd, 2);
    Serial.print(",");
    Serial.print("PPR:");
    Serial.print(pulsesPerRev);
    Serial.print(",");
    Serial.print("Sens:");
    Serial.println(sensitivityScale, 3);
}
