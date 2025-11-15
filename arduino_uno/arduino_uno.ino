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
 * - 4 potentiometers for real-time tuning
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

// Serial command buffer
char serialBuffer[SERIAL_BUFFER_SIZE];
int bufferIndex = 0;

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
float targetRPM = PRODUCTION_TARGET_RPM;
int pulsesPerRev = DEFAULT_PULSES_PER_REV; // Configurable pulses per revolution

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

    Serial.println(F("BLDC PID Controller Started"));
    Serial.println(F("Mode: Production (default)"));
    Serial.print(F("Target RPM: "));
    Serial.println(PRODUCTION_TARGET_RPM);
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
    Serial.println(pidOutput);
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
#if EMERGENCY_STOP_ENABLED
        Serial.print(F("Emergency Stop: "));
        Serial.println(emergencyStop ? F("ACTIVE") : F("Inactive"));
#endif
        Serial.print(F("Mode: "));
        if (serialTuningMode) Serial.println(F("Serial Tuning"));
        else if (tuningMode) Serial.println(F("Potentiometer Tuning"));
        else Serial.println(F("Production"));
        Serial.println();
    }
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
                Serial.print(F("Target RPM set to: "));
                Serial.println(targetRPM);
            } else if (paramName == "KP") {
                kp = value;
                serialTuningMode = true;
                Serial.print(F("Kp set to: "));
                Serial.println(kp);
            } else if (paramName == "KI") {
                ki = value;
                serialTuningMode = true;
                Serial.print(F("Ki set to: "));
                Serial.println(ki);
            } else if (paramName == "KD") {
                kd = value;
                serialTuningMode = true;
                Serial.print(F("Kd set to: "));
                Serial.println(kd);
            } else if (paramName == "PULSES") {
                int intValue = (int)value;
                if (intValue >= 1 && intValue <= 100) {
                    pulsesPerRev = intValue;
                    serialTuningMode = true;
                    Serial.print(F("Pulses per revolution set to: "));
                    Serial.println(pulsesPerRev);
                } else {
                    Serial.print(F("Invalid pulses per revolution value (1-100): "));
                    Serial.println(intValue);
                }
            } else {
                Serial.print(F("Unknown parameter: "));
                Serial.println(paramName);
            }
        } else {
            Serial.println(F("Invalid SET command format. Use: SET <PARAM> <VALUE>"));
        }
    } else if (command == "GET PARAMS") {
        printParameters();
    } else if (command == "SAVE") {
        saveParametersToEEPROM();
        Serial.println(F("Parameters saved to EEPROM"));
    } else if (command == "LOAD") {
        loadParametersFromEEPROM();
        Serial.println(F("Parameters loaded from EEPROM"));
    } else if (command == "HELP") {
        printHelp();
    } else if (command == "MODE PRODUCTION") {
        serialTuningMode = false;
        tuningMode = false;
        Serial.println(F("Switched to production mode"));
    } else if (command == "MODE SERIAL") {
        serialTuningMode = true;
        Serial.println(F("Switched to serial tuning mode"));
    } else if (command == "RESET INTEGRAL") {
        integral = 0.0;
        Serial.println(F("Integral term reset to 0"));
    } else {
        Serial.print(F("Unknown command: "));
        Serial.println(command);
        Serial.println(F("Type HELP for available commands"));
    }
}

// Save current parameters to EEPROM with basic validation
void saveParametersToEEPROM() {
    // Basic validation without string operations to save memory
    bool valid = true;
    if (targetRPM < 0 || targetRPM > 5000 || isnan(targetRPM)) valid = false;
    if (kp < 0 || kp > 10.0 || isnan(kp)) valid = false;
    if (ki < 0 || ki > 5.0 || isnan(ki)) valid = false;
    if (kd < 0 || kd > 1.0 || isnan(kd)) valid = false;
    if (pulsesPerRev < 1 || pulsesPerRev > 100) valid = false;

    if (!valid) {
        Serial.println(F("EEPROM save failed: invalid params"));
        return;
    }

    EEPROM.put(EEPROM_TARGET_RPM_ADDR, targetRPM);
    EEPROM.put(EEPROM_KP_ADDR, kp);
    EEPROM.put(EEPROM_KI_ADDR, ki);
    EEPROM.put(EEPROM_KD_ADDR, kd);
    EEPROM.put(EEPROM_PULSES_PER_REV_ADDR, pulsesPerRev);
}

// Load parameters from EEPROM
void loadParametersFromEEPROM() {
    float savedTargetRPM, savedKp, savedKi, savedKd;
    int savedPulsesPerRev;

    EEPROM.get(EEPROM_TARGET_RPM_ADDR, savedTargetRPM);
    EEPROM.get(EEPROM_KP_ADDR, savedKp);
    EEPROM.get(EEPROM_KI_ADDR, savedKi);
    EEPROM.get(EEPROM_KD_ADDR, savedKd);
    EEPROM.get(EEPROM_PULSES_PER_REV_ADDR, savedPulsesPerRev);

    // Check if EEPROM values are valid (not NaN or extreme values)
    if (!isnan(savedTargetRPM) && savedTargetRPM >= 0 && savedTargetRPM <= 5000) {
        targetRPM = savedTargetRPM;
        Serial.print(F("Loaded Target RPM: "));
        Serial.println(targetRPM);
    }

    if (!isnan(savedKp) && savedKp >= 0 && savedKp <= 10.0) {
        kp = savedKp;
        Serial.print(F("Loaded Kp: "));
        Serial.println(kp);
    }

    if (!isnan(savedKi) && savedKi >= 0 && savedKi <= 5.0) {
        ki = savedKi;
        Serial.print(F("Loaded Ki: "));
        Serial.println(ki);
    }

    if (!isnan(savedKd) && savedKd >= 0 && savedKd <= 1.0) {
        kd = savedKd;
        Serial.print(F("Loaded Kd: "));
        Serial.println(kd);
    }

    if (savedPulsesPerRev >= 1 && savedPulsesPerRev <= 100) {
        pulsesPerRev = savedPulsesPerRev;
        Serial.print(F("Loaded Pulses per Rev: "));
        Serial.println(pulsesPerRev);
    }
}

// Print help information
void printHelp() {
    Serial.println(F("=== BLDC PID Controller Commands ==="));
    Serial.println(F("SET TARGET <rpm>     - Set target RPM (0-5000)"));
    Serial.println(F("SET KP <value>       - Set proportional gain (0-10.0)"));
    Serial.println(F("SET KI <value>       - Set integral gain (0-5.0)"));
    Serial.println(F("SET KD <value>       - Set derivative gain (0-1.0)"));
    Serial.println(F("SET PULSES <value>   - Set pulses per revolution (1-100)"));
    Serial.println(F("GET PARAMS           - Display current parameters"));
    Serial.println(F("SAVE                 - Save parameters to EEPROM"));
    Serial.println(F("LOAD                 - Load parameters from EEPROM"));
    Serial.println(F("MODE PRODUCTION      - Switch to production mode"));
    Serial.println(F("MODE SERIAL          - Switch to serial tuning mode"));
    Serial.println(F("RESET INTEGRAL       - Reset integral term to 0"));
    Serial.println(F("HELP                 - Show this help message"));
    Serial.println();
    Serial.println(F("Modes: Mode switch LOW = Potentiometer, Serial MODE SERIAL = Serial, Default = Production"));
    Serial.println(F("Safety: Emergency stop after 5s no pulses, Soft-start enabled"));
}

// Print current parameters
void printParameters() {
    Serial.println(F("=== Current Parameters ==="));
    Serial.print(F("Target RPM: "));
    Serial.println(targetRPM);
    Serial.print(F("Kp: "));
    Serial.println(kp, 4);
    Serial.print(F("Ki: "));
    Serial.println(ki, 4);
    Serial.print(F("Kd: "));
    Serial.println(kd, 4);
    Serial.print(F("Pulses per Rev: "));
    Serial.println(pulsesPerRev);
    Serial.print(F("Current RPM: "));
    Serial.println(currentRPM, 1);
    Serial.print(F("PID Output: "));
    Serial.println(pidOutput, 2);
    Serial.print(F("Integral: "));
    Serial.println(integral, 2);

    if (serialTuningMode) {
        Serial.println(F("Mode: Serial Tuning"));
    } else if (tuningMode) {
        Serial.println(F("Mode: Potentiometer Tuning"));
    } else {
        Serial.println(F("Mode: Production"));
    }
    Serial.println();
}
