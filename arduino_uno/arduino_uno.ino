/**
 * BLDC Motor PID Controller
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
 *
 * Hardware Requirements:
 * - Arduino board (Uno, Mega, or similar)
 * - 3-Hall BLDC motor (such as 42BLF20-22.0223 or equivalent)
 * - BLDC motor controller (ESC) compatible with the motor
 * - Any one Hall sensor wire from the BLDC motor (Hall A, B, or C)
 * - Mode switch (jumper/digital input for tuning mode)
 *
 * Optional Hardware (for potentiometer tuning):
 * - 4 potentiometers for real-time tuning
 *
 * Author: AI Assistant
 * Date: November 2025
 */

// Include necessary libraries
#include <Arduino.h>
#include <EEPROM.h>

// Pin definitions
#define RPM_SENSOR_PIN      2   // Interrupt pin for BLDC Hall sensor (any Hall wire from motor)
#define PWM_OUTPUT_PIN      9   // PWM output to ESC
#define MODE_SWITCH_PIN     3   // Digital input for mode selection
#define POT_TARGET_RPM      A0  // Potentiometer for target RPM
#define POT_KP              A1  // Potentiometer for Kp gain
#define POT_KI              A2  // Potentiometer for Ki gain
#define POT_KD              A3  // Potentiometer for Kd gain

// Control parameters
#define CONTROL_LOOP_HZ     100 // Control loop frequency (100 Hz)
#define CONTROL_PERIOD_MS   (1000 / CONTROL_LOOP_HZ)

// RPM calculation parameters
#define PULSES_PER_REV      6   // Number of pulses per revolution (6 for 3-Hall BLDC motors like 42BLF20-22.0223)
#define RPM_CALC_INTERVAL   100 // RPM calculation interval in ms

// PID limits
#define PID_OUTPUT_MIN      -255 // Minimum PID output
#define PID_OUTPUT_MAX      255  // Maximum PID output
#define INTEGRAL_WINDUP_MIN -100 // Anti-windup integral minimum
#define INTEGRAL_WINDUP_MAX 100  // Anti-windup integral maximum

// Production mode default values (tune these during testing)
#define PRODUCTION_TARGET_RPM 1440.0
#define PRODUCTION_KP         0.5
#define PRODUCTION_KI         0.1
#define PRODUCTION_KD         0.01

// Serial command buffer
#define SERIAL_BUFFER_SIZE    64
char serialBuffer[SERIAL_BUFFER_SIZE];
int bufferIndex = 0;

// EEPROM addresses for storing parameters
#define EEPROM_TARGET_RPM_ADDR 0
#define EEPROM_KP_ADDR         4
#define EEPROM_KI_ADDR         8
#define EEPROM_KD_ADDR         12

// Global variables
volatile unsigned long pulseCount = 0;
unsigned long lastRPMCalcTime = 0;
float currentRPM = 0.0;
float targetRPM = PRODUCTION_TARGET_RPM;

// PID variables
float kp = PRODUCTION_KP;
float ki = PRODUCTION_KI;
float kd = PRODUCTION_KD;
float previousError = 0.0;
float integral = 0.0;
float pidOutput = 0.0;

// Mode selection
bool tuningMode = false;
bool serialTuningMode = false;

// Function prototypes
void rpmSensorISR();
float calculateRPM();
float readPotentiometer(int pin, float minVal, float maxVal);
void updatePIDGains();
float computePID(float error);
void outputToESC(int pwmValue);
void printToSerialPlotter();
void processSerialCommands();
void parseSerialCommand(String command);
void saveParametersToEEPROM();
void loadParametersFromEEPROM();
void printHelp();
void printParameters();

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

    // Load parameters from EEPROM
    loadParametersFromEEPROM();

    Serial.println("BLDC PID Controller Started");
    Serial.println("Mode: Production (default)");
    Serial.println("Target RPM: " + String(PRODUCTION_TARGET_RPM));
    Serial.println();
    printHelp();
}

void loop() {
    unsigned long currentTime = millis();

    // Process serial commands
    processSerialCommands();

    // Check mode switch and serial tuning mode
    tuningMode = digitalRead(MODE_SWITCH_PIN) == LOW;

    // Calculate RPM at regular intervals
    if (currentTime - lastRPMCalcTime >= RPM_CALC_INTERVAL) {
        currentRPM = calculateRPM();
        lastRPMCalcTime = currentTime;
    }

    // Update PID gains based on mode
    if (serialTuningMode) {
        // Serial tuning mode - parameters set via serial commands
        // Keep current values (set by serial commands)
    } else if (tuningMode) {
        // Potentiometer tuning mode
        updatePIDGains();
    } else {
        // Production mode - use hardcoded values
        targetRPM = PRODUCTION_TARGET_RPM;
        kp = PRODUCTION_KP;
        ki = PRODUCTION_KI;
        kd = PRODUCTION_KD;
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

    // Control loop timing
    delay(CONTROL_PERIOD_MS);
}

// Interrupt service routine for RPM sensor
void rpmSensorISR() {
    pulseCount++;
}

// Calculate RPM from pulse count
float calculateRPM() {
    static unsigned long lastPulseCount = 0;
    static unsigned long lastCalcTime = 0;

    unsigned long currentTime = millis();
    unsigned long timeDiff = currentTime - lastCalcTime;

    if (timeDiff >= RPM_CALC_INTERVAL) {
        unsigned long pulseDiff = pulseCount - lastPulseCount;

        // Calculate RPM: (pulses / time) * (60 seconds / pulses_per_rev)
        float rpm = (pulseDiff * 60000.0) / (timeDiff * PULSES_PER_REV);

        lastPulseCount = pulseCount;
        lastCalcTime = currentTime;

        return rpm;
    }

    return currentRPM; // Return previous value if not enough time has passed
}

// Read potentiometer and map to specified range
float readPotentiometer(int pin, float minVal, float maxVal) {
    int rawValue = analogRead(pin);
    return map(rawValue, 0, 1023, minVal * 100, maxVal * 100) / 100.0;
}

// Update PID gains from potentiometers (tuning mode only)
void updatePIDGains() {
    targetRPM = readPotentiometer(POT_TARGET_RPM, 0, 3000); // 0-3000 RPM range
    kp = readPotentiometer(POT_KP, 0, 2.0);                 // 0-2.0 Kp range
    ki = readPotentiometer(POT_KI, 0, 1.0);                 // 0-1.0 Ki range
    kd = readPotentiometer(POT_KD, 0, 0.1);                 // 0-0.1 Kd range
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

// Output PWM value to ESC
void outputToESC(int pwmValue) {
    analogWrite(PWM_OUTPUT_PIN, pwmValue);
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
    Serial.println(pidOutput);
}

// Process incoming serial commands
void processSerialCommands() {
    while (Serial.available() > 0) {
        char incomingChar = Serial.read();

        if (incomingChar == '\n' || incomingChar == '\r') {
            if (bufferIndex > 0) {
                serialBuffer[bufferIndex] = '\0'; // Null terminate
                parseSerialCommand(String(serialBuffer));
                bufferIndex = 0; // Reset buffer
            }
        } else if (bufferIndex < SERIAL_BUFFER_SIZE - 1) {
            serialBuffer[bufferIndex++] = incomingChar;
        }
    }
}

// Parse and execute serial commands
void parseSerialCommand(String command) {
    command.trim();
    command.toUpperCase();

    if (command.startsWith("SET ")) {
        String param = command.substring(4);
        int spaceIndex = param.indexOf(' ');

        if (spaceIndex > 0) {
            String paramName = param.substring(0, spaceIndex);
            String paramValue = param.substring(spaceIndex + 1);

            float value = paramValue.toFloat();

            if (paramName == "TARGET") {
                targetRPM = value;
                serialTuningMode = true;
                Serial.println("Target RPM set to: " + String(targetRPM));
            } else if (paramName == "KP") {
                kp = value;
                serialTuningMode = true;
                Serial.println("Kp set to: " + String(kp));
            } else if (paramName == "KI") {
                ki = value;
                serialTuningMode = true;
                Serial.println("Ki set to: " + String(ki));
            } else if (paramName == "KD") {
                kd = value;
                serialTuningMode = true;
                Serial.println("Kd set to: " + String(kd));
            } else {
                Serial.println("Unknown parameter: " + paramName);
            }
        } else {
            Serial.println("Invalid SET command format. Use: SET <PARAM> <VALUE>");
        }
    } else if (command == "GET PARAMS") {
        printParameters();
    } else if (command == "SAVE") {
        saveParametersToEEPROM();
        Serial.println("Parameters saved to EEPROM");
    } else if (command == "LOAD") {
        loadParametersFromEEPROM();
        Serial.println("Parameters loaded from EEPROM");
    } else if (command == "HELP") {
        printHelp();
    } else if (command == "MODE PRODUCTION") {
        serialTuningMode = false;
        tuningMode = false;
        Serial.println("Switched to production mode");
    } else if (command == "MODE SERIAL") {
        serialTuningMode = true;
        Serial.println("Switched to serial tuning mode");
    } else if (command == "RESET INTEGRAL") {
        integral = 0.0;
        Serial.println("Integral term reset to 0");
    } else {
        Serial.println("Unknown command: " + command);
        Serial.println("Type HELP for available commands");
    }
}

// Save current parameters to EEPROM
void saveParametersToEEPROM() {
    EEPROM.put(EEPROM_TARGET_RPM_ADDR, targetRPM);
    EEPROM.put(EEPROM_KP_ADDR, kp);
    EEPROM.put(EEPROM_KI_ADDR, ki);
    EEPROM.put(EEPROM_KD_ADDR, kd);
}

// Load parameters from EEPROM
void loadParametersFromEEPROM() {
    float savedTargetRPM, savedKp, savedKi, savedKd;

    EEPROM.get(EEPROM_TARGET_RPM_ADDR, savedTargetRPM);
    EEPROM.get(EEPROM_KP_ADDR, savedKp);
    EEPROM.get(EEPROM_KI_ADDR, savedKi);
    EEPROM.get(EEPROM_KD_ADDR, savedKd);

    // Check if EEPROM values are valid (not NaN or extreme values)
    if (!isnan(savedTargetRPM) && savedTargetRPM >= 0 && savedTargetRPM <= 5000) {
        targetRPM = savedTargetRPM;
        Serial.println("Loaded Target RPM: " + String(targetRPM));
    }

    if (!isnan(savedKp) && savedKp >= 0 && savedKp <= 10.0) {
        kp = savedKp;
        Serial.println("Loaded Kp: " + String(kp));
    }

    if (!isnan(savedKi) && savedKi >= 0 && savedKi <= 5.0) {
        ki = savedKi;
        Serial.println("Loaded Ki: " + String(ki));
    }

    if (!isnan(savedKd) && savedKd >= 0 && savedKd <= 1.0) {
        kd = savedKd;
        Serial.println("Loaded Kd: " + String(kd));
    }
}

// Print help information
void printHelp() {
    Serial.println("=== BLDC PID Controller Commands ===");
    Serial.println("SET TARGET <rpm>     - Set target RPM (0-5000)");
    Serial.println("SET KP <value>       - Set proportional gain (0-10.0)");
    Serial.println("SET KI <value>       - Set integral gain (0-5.0)");
    Serial.println("SET KD <value>       - Set derivative gain (0-1.0)");
    Serial.println("GET PARAMS           - Display current parameters");
    Serial.println("SAVE                 - Save parameters to EEPROM");
    Serial.println("LOAD                 - Load parameters from EEPROM");
    Serial.println("MODE PRODUCTION      - Switch to production mode");
    Serial.println("MODE SERIAL          - Switch to serial tuning mode");
    Serial.println("RESET INTEGRAL       - Reset integral term to 0");
    Serial.println("HELP                 - Show this help message");
    Serial.println();
    Serial.println("Current mode indicators:");
    Serial.println("- Mode switch LOW = Potentiometer tuning");
    Serial.println("- Serial MODE SERIAL = Serial tuning");
    Serial.println("- Default = Production mode");
}

// Print current parameters
void printParameters() {
    Serial.println("=== Current Parameters ===");
    Serial.println("Target RPM: " + String(targetRPM));
    Serial.println("Kp: " + String(kp, 4));
    Serial.println("Ki: " + String(ki, 4));
    Serial.println("Kd: " + String(kd, 4));
    Serial.println("Current RPM: " + String(currentRPM, 1));
    Serial.println("PID Output: " + String(pidOutput, 2));
    Serial.println("Integral: " + String(integral, 2));

    if (serialTuningMode) {
        Serial.println("Mode: Serial Tuning");
    } else if (tuningMode) {
        Serial.println("Mode: Potentiometer Tuning");
    } else {
        Serial.println("Mode: Production");
    }
    Serial.println();
}
