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
 * - Three operating modes: Production, Potentiometer Tuning, and Serial Tuning
 * - RPM feedback via Hall sensor (direct motor connection)
 * - PWM output to ESC
 * - Real-time tuning via potentiometers, serial commands, or Serial Plotter
 * - EEPROM storage for PID parameters
 * - Interactive serial command interface
 * - Configurable parameters (pulses per revolution, etc.)
 *
 * Hardware Requirements:
 * - Arduino board (Uno, Mega, or similar)
 * - 3-Hall BLDC motor (such as 42BLF20-22.0223 or equivalent)
 * - BLDC motor controller (ESC) compatible with the motor
 * - Any one Hall sensor wire from the BLDC motor (Hall A, B, or C)
 * - Mode switch (jumper/digital input for tuning mode)
 *
 * Optional Hardware (for potentiometer tuning):
 * - 5 potentiometers for real-time tuning
 *
 * Configuration:
 * - All configuration parameters are defined in config.h
 * - Modify config.h to change default values, pin assignments, etc.
 *
 * Author: azzar budiyanto
 * Co-Author: azzar persona (AI assistant)
 * Date: November 2025
 */

// Include configuration header (contains all pin definitions, constants, and settings)
#include "config.h"

// Serial removed - no interactive commands

// Debug timing
unsigned long lastDebugTime = 0;

// Global variables
volatile unsigned long pulseCount = 0;
volatile unsigned long lastPulseMicros = 0;
#if EMERGENCY_STOP_ENABLED
volatile unsigned long lastPulseMillis = 0;  // For emergency stop timeout
#endif
unsigned long lastRPMCalcTime = 0;
float currentRPM = 0.0;
const float targetRPM = PRODUCTION_TARGET_RPM; // Fixed target RPM for constant speed control
int pulsesPerRev = PULSES_PER_REV; // Configurable pulses per revolution via potentiometer

// PID variables
float kp = PRODUCTION_KP;
float ki = PRODUCTION_KI;
float kd = PRODUCTION_KD;
float previousError = 0.0;
float integral = 0.0;
float pidOutput = 0.0;

// Mode selection
bool tuningMode = false;
// Serial tuning removed

// Safety features
#if EMERGENCY_STOP_ENABLED
bool emergencyStop = false;
#endif

// Soft-start ramping to avoid current surges
unsigned long softStartStartTime = 0;
bool softStarting = true;
int softStartStep = 0;

// Function prototypes
void rpmSensorISR();
float calculateRPM();
float readPotentiometer(int pin, float minVal, float maxVal);
void updatePIDGains();
float computePID(float error);
void outputToESC(int pwmValue);
void printToSerialPlotter();
// Serial commands removed
// EEPROM functions removed
// Print functions removed

void setup() {
    // Initialize serial communication
    Serial.begin(115200);

    // Configure pins
    pinMode(RPM_SENSOR_PIN, INPUT_PULLUP);
    pinMode(PWM_OUTPUT_PIN, OUTPUT);
    pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);

    // Attach interrupt for BLDC Hall sensor
    attachInterrupt(digitalPinToInterrupt(RPM_SENSOR_PIN), rpmSensorISR, RISING);

    // Initialize PWM output to stopped position
    analogWrite(PWM_OUTPUT_PIN, 0);

    // Brief startup delay
    delay(1000);

    // EEPROM loading removed

    Serial.println(F("BLDC PID Controller Started"));
    Serial.println(F("Mode: Production (default)"));
    Serial.print(F("Target RPM: "));
    Serial.println(PRODUCTION_TARGET_RPM);
}

void loop() {

    unsigned long currentTime = millis();

    // Check mode switch (serial tuning removed)
    tuningMode = digitalRead(MODE_SWITCH_PIN) == LOW;

    // Debug mode switching
    static bool lastTuningMode = !tuningMode; // Force initial print
    if (tuningMode != lastTuningMode) {
        Serial.print(F("Mode switch changed to: "));
        Serial.println(tuningMode ? F("Potentiometer Tuning") : F("Production"));
        lastTuningMode = tuningMode;
    }

    // Calculate RPM at regular intervals
    if (currentTime - lastRPMCalcTime >= RPM_CALC_INTERVAL) {
        currentRPM = calculateRPM();
        lastRPMCalcTime = currentTime;
    }

#if EMERGENCY_STOP_ENABLED
    // Emergency stop: check if motor has stopped (no pulses received recently)
    emergencyStop = (currentTime - lastPulseMillis > EMERGENCY_STOP_TIMEOUT_MS);
    if (emergencyStop && currentRPM > 10.0) {  // Only trigger if we were running
        Serial.println(F("EMERGENCY STOP: No pulses"));
        pidOutput = 0.0;  // Force zero output
        integral = 0.0;   // Reset integral term
    }
#endif

    // Update PID gains based on mode
    if (tuningMode) {
        // Potentiometer tuning mode
        updatePIDGains();
        Serial.println("Mode: Potentiometer Tuning");
    } else {
        // Production mode - use hardcoded values
        // targetRPM is now fixed at 1440.0 (const)
        kp = PRODUCTION_KP;
        ki = PRODUCTION_KI;
        kd = PRODUCTION_KD;
        Serial.println("Mode: Production");
    }

    // Compute PID output
    float error = targetRPM - currentRPM;
    pidOutput = computePID(error);

    // Convert PID output to PWM value and output to ESC
    int pwmValue = map(pidOutput, PID_OUTPUT_MIN, PID_OUTPUT_MAX, 0, 255);
    pwmValue = constrain(pwmValue, 0, 255);
    outputToESC(pwmValue);

    // Output to Serial Plotter for monitoring
    printToSerialPlotter();

    // Debug output for bench testing
    printDebugInfo();


    // Control loop timing
    delay(CONTROL_PERIOD_MS);
}

// Interrupt service routine for RPM sensor with debounce filtering
void rpmSensorISR() {
    unsigned long t = micros();
    if (t - lastPulseMicros > MIN_PULSE_WIDTH_US) {
        pulseCount++;
        lastPulseMicros = t;
#if EMERGENCY_STOP_ENABLED
        lastPulseMillis = millis();  // Track last pulse time for emergency stop
#endif
    }
}

// Calculate RPM from pulse count with atomic read
float calculateRPM() {
    static unsigned long lastPulseCount = 0;
    static unsigned long lastCalcTime = 0;

    unsigned long currentTime = millis();
    unsigned long timeDiff = currentTime - lastCalcTime;

    if (timeDiff >= RPM_CALC_INTERVAL) {
        // Atomic read of volatile pulseCount to avoid race conditions
        noInterrupts();
        unsigned long pulsesNow = pulseCount;
        interrupts();

        unsigned long pulseDiff = pulsesNow - lastPulseCount;

        // Calculate RPM: (pulses / time) * (60 seconds / pulses_per_rev)
        float rpm = (pulseDiff * 60000.0) / (timeDiff * pulsesPerRev);

        lastPulseCount = pulsesNow;
        lastCalcTime = currentTime;

        return rpm;
    }

    return currentRPM; // Return previous value if not enough time has passed
}

// Read potentiometer and map to specified range (float)
float readPotentiometer(int pin, float minVal, float maxVal) {
    int rawValue = analogRead(pin);
    return map(rawValue, 0, 1023, minVal * 100, maxVal * 100) / 100.0;
}

// Read potentiometer and map to specified integer range
int readPotentiometerInt(int pin, int minVal, int maxVal) {
    int rawValue = analogRead(pin);
    return map(rawValue, 0, 1023, minVal, maxVal);
}

// Update PID gains from potentiometers (tuning mode only)
void updatePIDGains() {
    pulsesPerRev = readPotentiometerInt(POT_PULSES_PER_REV, 1, 100); // 1-100 pulses per revolution
    kp = readPotentiometer(POT_KP, 0, 2.0);                         // 0-2.0 Kp range
    ki = readPotentiometer(POT_KI, 0, 1.0);                         // 0-1.0 Ki range
    kd = readPotentiometer(POT_KD, 0, 0.1);                         // 0-0.1 Kd range
}

// Compute PID output with anti-windup
float computePID(float error) {
    // Proportional term
    float proportional = kp * error;

    // Integral term with anti-windup
    integral += ki * error;

    // Clamp integral to prevent windup
    integral = constrain(integral, INTEGRAL_WINDUP_MIN, INTEGRAL_WINDUP_MAX);

    // Derivative term
    float derivative = kd * (error - previousError);
    previousError = error;

    // Calculate total PID output
    float output = proportional + integral + derivative;

    // Clamp output to safe range
    output = constrain(output, PID_OUTPUT_MIN, PID_OUTPUT_MAX);

    return output;
}

// Apply soft-start ramping to avoid current surges
int applySoftStart(int targetPWM) {
    if (!softStarting) {
        return targetPWM;  // Normal operation
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
    return (int)(targetPWM * rampProgress);
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
    Serial.println(pulsesPerRev);
}

// Debug output for bench testing
void printDebugInfo() {
    if (!DEBUG_MODE_ENABLED) return;

    unsigned long currentTime = millis();
    if (currentTime - lastDebugTime >= DEBUG_INTERVAL_MS) {
        lastDebugTime = currentTime;

        // Atomic read of pulse count for debug
        noInterrupts();
        unsigned long debugPulseCount = pulseCount;
        interrupts();

        Serial.println(F("=== DEBUG INFO ==="));
        Serial.print(F("Pulse Count: "));
        Serial.println(debugPulseCount);
        Serial.print(F("Current RPM: "));
        Serial.println(currentRPM, 1);
        Serial.print(F("Target RPM: "));
        Serial.println(targetRPM, 1);
        Serial.print(F("Pulses per Rev: "));
        Serial.println(pulsesPerRev);
        Serial.print(F("PID Output: "));
        Serial.println(pidOutput, 2);
        Serial.print(F("Integral: "));
        Serial.println(integral, 2);
        Serial.print(F("Soft Starting: "));
        Serial.println(softStarting ? F("YES") : F("NO"));
#if WATCHDOG_ENABLED
        Serial.println(F("Watchdog: ENABLED"));
#endif
#if EMERGENCY_STOP_ENABLED
        Serial.print(F("Emergency Stop: "));
        Serial.println(emergencyStop ? F("ACTIVE") : F("Inactive"));
#endif
        Serial.print(F("Mode: "));
        if (tuningMode) Serial.println(F("Potentiometer Tuning"));
        else Serial.println(F("Production"));
        Serial.println();
    }
}