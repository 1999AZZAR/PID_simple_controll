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
 * - EEPROM storage for PID parameters
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
#include <EEPROM.h>

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
bool motorEnabled = true;
bool autoTuneMode = false;

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
void saveParametersToEEPROM();
void loadParametersFromEEPROM();

// EEPROM addresses
#define EEPROM_KP_ADDR         0
#define EEPROM_KI_ADDR         4
#define EEPROM_KD_ADDR         8
#define EEPROM_TARGET_RPM_ADDR 12
#define EEPROM_PULSES_PER_REV_ADDR 16
#define EEPROM_CHECKSUM_ADDR   20

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

    // Load parameters from EEPROM
    loadParametersFromEEPROM();

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
    if (motorEnabled && !autoTuneMode) {
        float error = targetRPM - currentRPM;
        pidOutput = computePID(error);

        // Convert PID output to PWM value and output to ESC
        int pwmValue = map(pidOutput, PID_OUTPUT_MIN, PID_OUTPUT_MAX, 0, 255);
        pwmValue = constrain(pwmValue, 0, 255);
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
        Serial.print(F("Kp set to: "));
        Serial.println(kp, 4);
        saveParametersToEEPROM();

    } else if (command.startsWith("SET_KI ")) {
        ki = command.substring(7).toFloat();
        Serial.print(F("Ki set to: "));
        Serial.println(ki, 4);
        saveParametersToEEPROM();

    } else if (command.startsWith("SET_KD ")) {
        kd = command.substring(7).toFloat();
        Serial.print(F("Kd set to: "));
        Serial.println(kd, 4);
        saveParametersToEEPROM();

    } else if (command.startsWith("SET_TARGET_RPM ")) {
        targetRPM = command.substring(14).toFloat();
        Serial.print(F("Target RPM set to: "));
        Serial.println(targetRPM);
        saveParametersToEEPROM();

    } else if (command.startsWith("SET_PULSES_PER_REV ")) {
        pulsesPerRev = command.substring(18).toInt();
        Serial.print(F("Pulses per revolution set to: "));
        Serial.println(pulsesPerRev);
        saveParametersToEEPROM();

    } else if (command.startsWith("ENABLE_MOTOR ")) {
        motorEnabled = (command.substring(13) == "1");
        Serial.print(F("Motor "));
        Serial.println(motorEnabled ? F("enabled") : F("disabled"));

    } else if (command.startsWith("AUTO_TUNE ")) {
        autoTuneMode = (command.substring(10) == "1");
        if (autoTuneMode) {
            // Reset PID state for auto-tuning
            integral = 0;
            previousError = 0;
            Serial.println(F("Auto-tune mode enabled"));
        } else {
            Serial.println(F("Auto-tune mode disabled"));
        }

    } else if (command == "RESET_INTEGRAL") {
        integral = 0;
        Serial.println(F("Integral reset"));

    } else if (command == "SAVE_PARAMETERS") {
        saveParametersToEEPROM();
        Serial.println(F("Parameters saved to EEPROM"));

    } else if (command == "LOAD_PARAMETERS") {
        loadParametersFromEEPROM();
        Serial.println(F("Parameters loaded from EEPROM"));

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
    Serial.print(targetRPM);
    Serial.print(F(","));
    Serial.print(currentRPM);
    Serial.print(F(","));
    Serial.print(targetRPM - currentRPM);
    Serial.print(F(","));
    Serial.print(pidOutput);
    Serial.print(F(","));
    Serial.print(kp, 4);
    Serial.print(F(","));
    Serial.print(ki, 4);
    Serial.print(F(","));
    Serial.print(kd, 4);
    Serial.print(F(","));
    Serial.print(pulsesPerRev);
    Serial.print(F(","));
    Serial.print(motorEnabled ? 1 : 0);
    Serial.print(F(","));
    Serial.print(autoTuneMode ? 1 : 0);
    Serial.println();
}

// Save parameters to EEPROM
void saveParametersToEEPROM() {
    EEPROM.put(EEPROM_KP_ADDR, kp);
    EEPROM.put(EEPROM_KI_ADDR, ki);
    EEPROM.put(EEPROM_KD_ADDR, kd);
    EEPROM.put(EEPROM_TARGET_RPM_ADDR, targetRPM);
    EEPROM.put(EEPROM_PULSES_PER_REV_ADDR, pulsesPerRev);

    // Calculate and save checksum
    float checksum = kp + ki + kd + targetRPM + pulsesPerRev;
    EEPROM.put(EEPROM_CHECKSUM_ADDR, checksum);
}

// Load parameters from EEPROM
void loadParametersFromEEPROM() {
    float savedKp, savedKi, savedKd, savedTargetRPM, savedChecksum;
    int savedPulsesPerRev;

    EEPROM.get(EEPROM_KP_ADDR, savedKp);
    EEPROM.get(EEPROM_KI_ADDR, savedKi);
    EEPROM.get(EEPROM_KD_ADDR, savedKd);
    EEPROM.get(EEPROM_TARGET_RPM_ADDR, savedTargetRPM);
    EEPROM.get(EEPROM_PULSES_PER_REV_ADDR, savedPulsesPerRev);
    EEPROM.get(EEPROM_CHECKSUM_ADDR, savedChecksum);

    // Verify checksum
    float calculatedChecksum = savedKp + savedKi + savedKd + savedTargetRPM + savedPulsesPerRev;

    if (abs(calculatedChecksum - savedChecksum) < 0.01) {
        // Checksum valid, load parameters
        kp = savedKp;
        ki = savedKi;
        kd = savedKd;
        targetRPM = savedTargetRPM;
        pulsesPerRev = savedPulsesPerRev;
        Serial.println(F("Parameters loaded from EEPROM"));
    } else {
        // Checksum invalid, use defaults
        Serial.println(F("EEPROM checksum invalid, using default parameters"));
    }
}
