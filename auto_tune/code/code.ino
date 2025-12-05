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

// Include shared common headers
#include "pid_common.h"
#include "rpm_common.h"
#include "isr_common.h"

// Global variables
volatile unsigned long pulseInterval = 0;  // Time interval between pulses in microseconds
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
int lastPWMValue = 0; // Last PWM value for monitoring

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

        // Use standard PID control for all speeds
        pidOutput = computePID(error);

        // Convert PID output to PWM value with full range for better accuracy
        int pwmValue;
        static unsigned long emergencyStartTime = 0;

        if (abs(error) > 2000) {  // If error > 2000 RPM, emergency stop
            if (emergencyStartTime == 0) {
                emergencyStartTime = millis();
                Serial.println("EMERGENCY STOP: Motor out of control!");
            }
            pwmValue = 0;  // Cut power completely for 2 seconds
        } else {
            // Convert PID output to PWM value and output to ESC
            // Use wider PWM range for better control authority: 0-255 full range
            pwmValue = map(pidOutput, PID_OUTPUT_MIN, PID_OUTPUT_MAX, 0, 255);
            pwmValue = constrain(pwmValue, 0, 255);
        }

        // Reset emergency timer after 2 seconds regardless of current error state
        if (emergencyStartTime > 0 && millis() - emergencyStartTime > 2000) {
            emergencyStartTime = 0;  // Reset emergency after 2 seconds
        }

        // Temporarily disable hysteresis for debugging
        lastPWMValue = pwmValue;
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
    unsigned long currentMicros = micros();
    rpmSensorISR_common(currentMicros, lastPulseMicros, pulseInterval, MIN_PULSE_WIDTH_US);
}

// Calculate RPM from pulse interval with timeout detection and moving average filtering
float calculateRPM() {
    static unsigned long lastCalcTime = 0;
    unsigned long currentTime = micros();

    // Check if enough time has passed for RPM calculation interval
    if (currentTime - lastCalcTime >= RPM_CALC_INTERVAL * 1000UL) {
        float rpm = 0.0;

        // Timeout check: if no pulses received within timeout period, motor is stopped
        if (currentTime - lastPulseMicros > RPM_TIMEOUT_US) {
            rpm = 0.0; // Motor stopped
        } else {
            // Atomic read of volatile pulseInterval to avoid race conditions
            noInterrupts();
            unsigned long interval = pulseInterval;
            interrupts();

            // Calculate RPM using period measurement: RPM = (60,000,000) / (interval_μs * pulses_per_rev)
            // Use sequential division to avoid intermediate multiplication overflow
            if (interval > 0) {
                rpm = 60000000.0 / interval / pulsesPerRev;
            }
        }

        // Apply moving average filter for noise reduction and stability
        updateMovingAverage(rpmFiltered, rpm, rpmHistory, rpmHistoryIndex, RPM_FILTER_SIZE);

        lastCalcTime = currentTime;
        currentRPM = rpmFiltered;

        return rpmFiltered;
    }

    return currentRPM; // Return previous filtered value if not enough time has passed
}

// Compute PID output using shared function with overflow protection
float computePID(float error) {
    return computePID_float(error, integral, previousError, kp, ki, kd,
                           INTEGRAL_WINDUP_MIN, INTEGRAL_WINDUP_MAX,
                           PID_OUTPUT_MIN, PID_OUTPUT_MAX);
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
    // Temporarily disable soft start for debugging
    // int safePWM = applySoftStart(pwmValue);
    // analogWrite(PWM_OUTPUT_PIN, safePWM);

    // Direct PWM output for testing
    analogWrite(PWM_OUTPUT_PIN, pwmValue);
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
    Serial.print(lastPWMValue);
    Serial.print(F(","));
    Serial.print(pulsesPerRev);
    Serial.print(F(","));
    Serial.println(motorEnabled ? 1 : 0);
}

