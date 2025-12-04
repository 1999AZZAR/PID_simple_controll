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

// Moving average filter for RPM smoothing and accuracy
#define RPM_FILTER_SIZE 5
float rpmHistory[RPM_FILTER_SIZE] = {0};
int rpmHistoryIndex = 0;
float rpmFiltered = 0.0;

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
    pinMode(RPM_SENSOR_PIN, INPUT_PULLUP);
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

// Calculate RPM from pulse count with microsecond precision and moving average filtering
float calculateRPM() {
    static unsigned long lastPulseCount = 0;
    static unsigned long lastCalcTime = 0;

    unsigned long currentTime = micros();  // Use micros for maximum precision
    unsigned long timeDiff = currentTime - lastCalcTime;

    if (timeDiff >= RPM_CALC_INTERVAL * 1000UL) {  // Convert ms to microseconds
        // Atomic read of volatile pulseCount to avoid race conditions
        noInterrupts();
        unsigned long pulsesNow = pulseCount;
        interrupts();

        unsigned long pulseDiff = pulsesNow - lastPulseCount;

        float rpm = 0.0;
        if (pulseDiff > 0) {
            // Calculate RPM with microsecond precision: (pulses / time_seconds) * (60 / pulses_per_rev)
            float timeSeconds = timeDiff / 1000000.0;
            rpm = (pulseDiff / timeSeconds) * (60.0 / pulsesPerRev);
        }
        // For very low RPM (< 50), if no pulses detected, assume stopped
        else if (currentRPM > 50) {
            rpm = 0.0; // Motor stopped
        }

        // Apply moving average filter for noise reduction and stability
        rpmHistory[rpmHistoryIndex] = rpm;
        rpmHistoryIndex = (rpmHistoryIndex + 1) % RPM_FILTER_SIZE;

        // Calculate filtered RPM
        float sum = 0;
        for(int i = 0; i < RPM_FILTER_SIZE; i++) {
            sum += rpmHistory[i];
        }
        rpmFiltered = sum / RPM_FILTER_SIZE;

        lastPulseCount = pulsesNow;
        lastCalcTime = currentTime;
        currentRPM = rpmFiltered;

        return rpmFiltered;
    }

    return currentRPM; // Return previous filtered value if not enough time has passed
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

    } else if (command == "GET_STATUS") {
        sendStatusData();

    } else if (command == "TEST_PPR") {
        Serial.println(F("=== PPR TEST MODE ==="));
        Serial.print(F("Configured PPR: "));
        Serial.println(pulsesPerRev);

        // Count pulses for 5 seconds
        unsigned long startTime = millis();
        unsigned long startPulses = pulseCount;
        Serial.println(F("Counting pulses for 5 seconds..."));

        while (millis() - startTime < 5000) {
            delay(1000); // Update every second
            Serial.print(F("Elapsed: "));
            Serial.print((millis() - startTime) / 1000);
            Serial.print(F("s, Pulses: "));
            Serial.println(pulseCount - startPulses);
        }

        unsigned long totalPulses = pulseCount - startPulses;
        float measuredRPM = (totalPulses * 60.0) / (5.0 * pulsesPerRev);  // Simplified formula for 5 seconds
        Serial.print(F("Total pulses counted: "));
        Serial.println(totalPulses);
        Serial.print(F("Calculated RPM (5s average): "));
        Serial.println(measuredRPM, 2);
        Serial.println(F("If motor was spinning at known RPM, compare this value."));
        Serial.println(F("Formula: RPM = (pulses * 60) / (time_seconds * PPR)"));
        Serial.println(F("===================================="));

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

