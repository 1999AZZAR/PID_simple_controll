/*
 * BLDC Motor Simulator for ESP32-C3
 *
 * Simulates a BLDC motor + ESC response to test the main PID controller.
 *
 * HARDWARE CONNECTIONS:
 * ---------------------
 * Controller ESP32 Pin 1 (PWM Out) ----> Simulator ESP32 Pin 0 (PWM In)
 * Controller ESP32 Pin 0 (RPM In)  <---- Simulator ESP32 Pin 1 (RPM Out)
 * Controller GND                   ----> Simulator GND
 *
 * NO VOLTAGE DIVIDER NEEDED! (Both are 3.3V logic)
 */

#include <Arduino.h>

// Pins match the crossed connection from Controller
#define PWM_INPUT_PIN 0  // Receives PWM from Controller (GPIO 0)
#define RPM_OUTPUT_PIN 1 // Sends RPM pulses to Controller (GPIO 1)

// Motor Physics Configuration
#define MAX_RPM 3000.0    // Maximum RPM at 100% throttle
#define PULSES_PER_REV 4  // 8-pole motor (4 pulses per rev)
#define INERTIA 0.02      // Rotor mass simulation (Lower = Heavier/Slower to spin up)
#define NOISE_AMOUNT 15.0 // Random RPM jitter amplitude

float currentRPM = 0.0;
unsigned long lastDebugTime = 0;

void setup() {
    Serial.begin(115200);

    // Configure pins
    pinMode(PWM_INPUT_PIN, INPUT);
    pinMode(RPM_OUTPUT_PIN, OUTPUT);

    Serial.println("BLDC Motor Simulator Started");
    Serial.println("Waiting for PWM signal...");
}

void loop() {
    // 1. Measure Input PWM (Throttle)
    // Measures the HIGH pulse duration in microseconds
    // Timeout 25ms (handles down to 40Hz PWM)
    unsigned long highTime = pulseIn(PWM_INPUT_PIN, HIGH, 25000);
    unsigned long lowTime = pulseIn(PWM_INPUT_PIN, LOW, 25000);
    unsigned long period = highTime + lowTime;

    float dutyCycle = 0.0;

    // Calculate Duty Cycle (0.0 to 1.0)
    if (period > 0) {
        dutyCycle = (float)highTime / (float)period;
    } else {
        // If timeout or no signal, assume 0 throttle
        dutyCycle = 0.0;
    }

    // 2. Simulate Physics (Inertia & Response)
    float targetRPM = dutyCycle * MAX_RPM;

    // Apply inertia (Low-pass filter physics)
    // NewRPM = OldRPM * (1-k) + Target * k
    currentRPM = (currentRPM * (1.0 - INERTIA)) + (targetRPM * INERTIA);

    // 3. Generate RPM Output Pulses
    // We simulate the Hall sensor pulses
    if (currentRPM < 50) {
        noTone(RPM_OUTPUT_PIN); // Motor stopped
    } else {
        // Add some realistic noise/jitter
        float jitter = random(-NOISE_AMOUNT, NOISE_AMOUNT);
        float simulatedRPM = currentRPM + jitter;

        // Calculate Pulse Frequency
        // Freq = (RPM * PulsesPerRev) / 60
        // Example: 1440 RPM * 4 / 60 = 96 Hz
        long frequency = (long)((simulatedRPM * PULSES_PER_REV) / 60.0);

        // Use tone() to generate square wave
        if (frequency > 0) {
            tone(RPM_OUTPUT_PIN, frequency);
        }
    }

    // 4. Debug Output (every 100ms)
    if (millis() - lastDebugTime > 100) {
        Serial.print("Input PWM: ");
        Serial.print(dutyCycle * 100.0, 1);
        Serial.print("% | Target: ");
        Serial.print(targetRPM, 0);
        Serial.print(" RPM | Simulated: ");
        Serial.print(currentRPM, 0);
        Serial.println(" RPM");

        lastDebugTime = millis();
    }

    // Small loop delay to simulate processing time
    delay(5);
}
