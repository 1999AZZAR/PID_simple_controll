/**
 * BLDC Motor PID Controller - Arduino Uno Version (Python Configurable)
 *
 * This Arduino sketch implements a PID controller to maintain a BLDC motor
 * at configurable RPM, with all parameters adjustable via Python GUI.
 *
 * Motor Compatibility:
 * - Designed for 3-Hall BLDC motors (such as 42BLF20-22.0223)
 * - Works with any BLDC motor that has 3 built-in Hall effect sensors
 * - Hall sensors provide 6 pulses per electrical revolution
 * - Compatible with standard BLDC motor controllers (ESC)
 *
 * Features:
 * - PID control with anti-windup protection
 * - Serial communication with Python GUI for real-time configuration
 * - Auto PID tuning support (Python-side calculation)
 * - RPM feedback via Hall sensor (direct motor connection)
 * - PWM output to ESC
 * - Real-time monitoring data output
 * - All parameters controlled by Python GUI (no persistent storage)
 * - Configurable parameters (pulses per revolution, target RPM, etc.)
 *
 * Hardware Requirements:
 * - Arduino board (Uno, Mega, or similar)
 * - 3-Hall BLDC motor (such as 42BLF20-22.0223 or equivalent)
 * - BLDC motor controller (ESC) compatible with the motor
 * - Any one Hall sensor wire from the BLDC motor (Hall A, B, or C)
 *
 * Configuration:
 * - All configuration parameters are defined in config.h
 * - Parameters can be adjusted in real-time via Python GUI
 *
 * Author: azzar budiyanto
 * Co-Author: azzar persona (AI assistant)
 * Date: December 2025
 */

// Include configuration header (contains all pin definitions, constants, and settings)
#include "config.h"

// Global variables
volatile unsigned long pulseCount = 0;
volatile unsigned long lastPulseMicros = 0;
unsigned long lastRPMCalcTime = 0;
float currentRPM = 0.0;
float targetRPM = DEFAULT_TARGET_RPM;
int pulsesPerRev = DEFAULT_PULSES_PER_REV;

// PID variables
float kp = DEFAULT_KP;
float ki = DEFAULT_KI;
float kd = DEFAULT_KD;
float previousError = 0.0;
float integral = 0.0;
float pidOutput = 0.0;

// Serial communication
String inputString = "";
boolean stringComplete = false;
unsigned long lastSerialSend = 0;

// Control state
bool motorEnabled = false; // Start disabled for safety

// Soft-start ramping to avoid current surges
unsigned long softStartStartTime = 0;
bool softStarting = true;
int softStartStep = 0;

// Function prototypes
void rpmSensorISR();
float calculateRPM();
float computePID(float error);
void outputToESC(int pwmValue);
void processSerialCommand(String command);
void sendStatusData();

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    inputString.reserve(200);

    // Configure pins
    pinMode(RPM_SENSOR_PIN, INPUT_PULLUPwindow.loadPackages not available quick-purchase:1:83
    );
    pinMode(PWM_OUTPUT_PIN, OUTPUT);

    // Attach interrupt for BLDC Hall sensor
    attachInterrupt(digitalPinToInterrupt(RPM_SENSOR_PIN), rpmSensorISR, RISING);

    // Initialize PWM output to stopped position
    analogWrite(PWM_OUTPUT_PIN, 0);

    // Brief startup delay
    delay(1000);

    Serial.println(F("BLDC PID Controller Started - Python Configurable"));
    Serial.println(F("Ready for serial commands"));
    Serial.print(F("Target RPM: "));
    Serial.println(targetRPM);
    Serial.print(F("Kp: "));
    Serial.println(kp, 4);
    Serial.print(F("Ki: "));
    Serial.println(ki, 4);
    Serial.print(F("Kd: "));
    Serial.println(kd, 4);
    Serial.flush();  // Ensure startup messages are sent
}

void loop() {
    unsigned long currentTime = millis();

    // Process serial commands
    if (stringComplete) {
        processSerialCommand(inputString);
        inputString = "";
        stringComplete = false;
    }

    // Calculate RPM at regular intervals
    if (currentTime - lastRPMCalcTime >= RPM_CALC_INTERVAL) {
        currentRPM = calculateRPM();
        lastRPMCalcTime = currentTime;
    }

    // Compute PID output if motor is enabled
    if (motorEnabled) {
        float error = targetRPM - currentRPM;

        if (autoTuneMode) {
            // During auto-tune: Use P-only control to test different Kp values
            // Ki and Kd are set to 0 by the GUI before starting auto-tune
            pidOutput = kp * error;  // Simple proportional control only
        } else {
            // Special handling for very low RPM targets (< 50 RPM)
            if (targetRPM < 50) {
                // Use simplified proportional-only control for low speeds
                float lowSpeedKp = 2.0; // Higher proportional gain for low speeds
                pidOutput = lowSpeedKp * error;

                // Add small integral term for steady state accuracy
                static float lowSpeedIntegral = 0;
                lowSpeedIntegral += 0.01 * error;
                lowSpeedIntegral = constrain(lowSpeedIntegral, -50, 50);
                pidOutput += lowSpeedIntegral;

                pidOutput = constrain(pidOutput, 0, 200); // Positive only for low speeds
            } else {
                // Normal PID control for higher speeds
                pidOutput = computePID(error);
            }
        }

        // Convert PID output to PWM value with better low-speed control
        int pwmValue;
        if (targetRPM < 50) {
            // Direct mapping for low speeds (more resolution)
            pwmValue = constrain(pidOutput, 20, 80); // Narrow range for fine control
        } else {
            // Standard mapping for normal speeds
            pwmValue = map(pidOutput, PID_OUTPUT_MIN, PID_OUTPUT_MAX, 20, 255);
            pwmValue = constrain(pwmValue, 0, 255);
        }

        outputToESC(pwmValue);

        // Debug output every 500ms for troubleshooting
        static unsigned long lastDebug = 0;
        if (millis() - lastDebug > 500) {
            Serial.print(F("DEBUG: Mode="));
            Serial.print(F(", Target="));
            Serial.print(targetRPM);
            Serial.print(F(", Current="));
            Serial.print(currentRPM);
            Serial.print(F(", PWM="));
            Serial.println(pwmValue);
            lastDebug = millis();
        }
    } else if (!motorEnabled) {
        // Motor disabled - stop immediately
        analogWrite(PWM_OUTPUT_PIN, 0);
        pidOutput = 0;
    }

    // Send status data to Python GUI at regular intervals
    if (currentTime - lastSerialSend >= SERIAL_SEND_INTERVAL) {
        sendStatusData();
        lastSerialSend = currentTime;
    }

    // Debug: Send heartbeat every 2 seconds
    static unsigned long lastHeartbeat = 0;
    if (currentTime - lastHeartbeat >= 2000) {
        Serial.println(F("HEARTBEAT"));
        lastHeartbeat = currentTime;
    }

    // Control loop timing
    delay(CONTROL_PERIOD_MS);
}

void serialEvent() {
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        if (inChar == '\n') {
            stringComplete = true;
        } else {
            inputString += inChar;
        }
    }
}

// Interrupt service routine for RPM sensor with debounce filtering
void rpmSensorISR() {
    unsigned long t = micros();
    if (t - lastPulseMicros > MIN_PULSE_WIDTH_US) {
        pulseCount++;
        lastPulseMicros = t;
    }
}

// Calculate RPM from pulse count with atomic read
float calculateRPM() {
    static unsigned long lastPulseCount = 0;
    static unsigned long lastCalcTime = 0;
    static float filteredRPM = 0.0; // Low-pass filter for stability

    unsigned long currentTime = millis();
    unsigned long timeDiff = currentTime - lastCalcTime;

    if (timeDiff >= RPM_CALC_INTERVAL) {
        // Atomic read of volatile pulseCount to avoid race conditions
        noInterrupts();
        unsigned long pulsesNow = pulseCount;
        interrupts();

        unsigned long pulseDiff = pulsesNow - lastPulseCount;

        float rpm = 0.0;
        if (pulseDiff > 0) {
            // Calculate RPM: (pulses / time) * (60 seconds / pulses_per_rev)
            rpm = (pulseDiff * 60000.0) / (timeDiff * pulsesPerRev);
        }
        // For very low RPM (< 50), if no pulses detected, assume stopped
        else if (currentRPM > 50) {
            rpm = 0.0; // Motor stopped
        }

        // Apply low-pass filter for stability (alpha = 0.3)
        filteredRPM = 0.3 * rpm + 0.7 * filteredRPM;

        lastPulseCount = pulsesNow;
        lastCalcTime = currentTime;
        currentRPM = filteredRPM;

        return filteredRPM;
    }

    return currentRPM; // Return filtered value if not enough time has passed
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

// Process serial commands from Python GUI
void processSerialCommand(String command) {
    command.trim();

    if (command.startsWith("SET_KP ")) {
        kp = command.substring(7).toFloat();
        Serial.print(F("✓ Kp set to: "));
        Serial.println(kp, 4);

    } else if (command.startsWith("SET_KI ")) {
        ki = command.substring(7).toFloat();
        Serial.print(F("✓ Ki set to: "));
        Serial.println(ki, 4);

    } else if (command.startsWith("SET_KD ")) {
        kd = command.substring(7).toFloat();
        Serial.print(F("✓ Kd set to: "));
        Serial.println(kd, 4);

    } else if (command.startsWith("SET_TARGET_RPM ")) {
        targetRPM = command.substring(15).toFloat();
        Serial.print(F("Target RPM set to: "));
        Serial.println(targetRPM);

    } else if (command.startsWith("SET_PULSES_PER_REV ")) {
        pulsesPerRev = command.substring(19).toInt();
        Serial.print(F("Pulses per revolution set to: "));
        Serial.println(pulsesPerRev);

    } else if (command.startsWith("ENABLE_MOTOR ")) {
        motorEnabled = (command.substring(13) == "1");
        Serial.print(F("Motor "));
        Serial.println(motorEnabled ? F("enabled") : F("disabled"));
        // Immediately set PWM to 0 if disabling motor
        if (!motorEnabled) {
            analogWrite(PWM_OUTPUT_PIN, 0);
            pidOutput = 0;
            integral = 0; // Reset integral when disabling
            previousError = 0; // Reset derivative term
            // Also reset any auto-tune specific variables
            }

    } else if (command == "FORCE_STOP") {
        // Emergency stop - force motor to stop completely
        motorEnabled = false;
        analogWrite(PWM_OUTPUT_PIN, 0);
        pidOutput = 0;
        integral = 0;
        previousError = 0;
        Serial.println(F("FORCE STOP: Motor completely disabled"));

    } else if (command.startsWith("SET_PWM ")) {
        if (autoTuneMode) {
            // Direct PWM control during auto-tune (computer-controlled)
            int pwm_value = command.substring(8).toInt();
            pwm_value = constrain(pwm_value, 0, 255);
            analogWrite(PWM_OUTPUT_PIN, pwm_value);
            Serial.print(F("PWM set to: "));
            Serial.println(pwm_value);
        } else {
            Serial.println(F("ERROR: SET_PWM only allowed in auto-tune mode"));
        }

    } else if (command.startsWith("AUTO_TUNE ")) {
        autoTuneMode = (command.substring(10) == "1");
        if (autoTuneMode) {
            // Reset PID state for auto-tuning
            integral = 0;
            previousError = 0;
            Serial.println(F("Auto-tune mode enabled - computer control"));
        } else {
            Serial.println(F("✓ Auto-tune mode disabled - using PID control"));
        }

    } else if (command == "RESET_INTEGRAL") {
        integral = 0;
        Serial.println(F("Integral reset"));

    } else if (command == "GET_STATUS") {
        sendStatusData();

    } else if (command == "VERIFY_TUNED_VALUES") {
        Serial.println(F("=== TUNED PID VALUES VERIFICATION ==="));
        Serial.print(F("Current Kp: "));
        Serial.println(kp, 4);
        Serial.print(F("Current Ki: "));
        Serial.println(ki, 4);
        Serial.print(F("Current Kd: "));
        Serial.println(kd, 4);
        Serial.print(F("Auto-tune mode: "));
        Serial.println(F("===================================="));

    } else if (command == "DIAGNOSTICS") {
        Serial.println(F("=== ARDUINO DIAGNOSTICS ==="));
        Serial.print(F("Motor Enabled: "));
        Serial.println(motorEnabled ? F("YES") : F("NO"));
        Serial.print(F("Auto-tune Mode: "));
        Serial.print(F("Current RPM: "));
        Serial.println(currentRPM, 1);
        Serial.print(F("Target RPM: "));
        Serial.println(targetRPM, 1);
        Serial.print(F("PID Output: "));
        Serial.println(pidOutput, 2);
        Serial.print(F("Pulse Count: "));
        Serial.println(pulseCount);
        Serial.println(F("============================"));

    } else if (command == "SEND_STATUS") {
        // Force send STATUS data for debugging
        Serial.println(F("Force sending STATUS data..."));
        sendStatusData();

    } else if (command == "FORCE_STATUS") {
        Serial.print(F("STATUS:"));
        Serial.print(millis());
        Serial.print(F(","));
        Serial.print(targetRPM, 0);
        Serial.print(F(","));
        Serial.print(currentRPM, 1);
        Serial.print(F(","));
        Serial.print(targetRPM - currentRPM, 1);
        Serial.print(F(","));
        Serial.print(pidOutput, 1);
        Serial.print(F(","));
        Serial.print(kp, 4);
        Serial.print(F(","));
        Serial.print(ki, 4);
        Serial.print(F(","));
        Serial.print(kd, 4);
        Serial.print(F(","));
        Serial.print(pulsesPerRev);
        Serial.print(F(","));
        Serial.println(motorEnabled ? 1 : 0);

    } else if (command == "RESET_CONTROLLER") {
        // Reset all control variables
        integral = 0;
        previousError = 0;
        pidOutput = 0;
        softStarting = true;
        softStartStartTime = 0;
        Serial.println(F("Controller reset"));

    } else {
        Serial.print(F("Unknown command: "));
        Serial.println(command);
    }
}

// Send status data to Python GUI
void sendStatusData() {
    Serial.print(F("STATUS:"));
    Serial.print(millis());
    Serial.print(F(","));
    Serial.print(targetRPM, 0);
    Serial.print(F(","));
    Serial.print(currentRPM, 1);
    Serial.print(F(","));
    Serial.print(targetRPM - currentRPM, 1);
    Serial.print(F(","));
    Serial.print(pidOutput, 1);
    Serial.print(F(","));
    Serial.print(kp, 4);
    Serial.print(F(","));
    Serial.print(ki, 4);
    Serial.print(F(","));
    Serial.print(kd, 4);
    Serial.print(F(","));
    Serial.print(pulsesPerRev);
    Serial.print(F(","));
    Serial.println(motorEnabled ? 1 : 0);
}

