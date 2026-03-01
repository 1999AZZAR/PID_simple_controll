/**
 * BLDC Motor PID Controller - ESP32-C3 SuperMini Production Version
 *
 * High-performance PID controller for ESP32-C3
 * Leverages hardware peripherals (PCNT, LEDC) and FPU for ultra-stable control
 *
 * Hardware Connections:
 * - 5V -> 5V (Vin)
 * - GND -> GND
 * - GPIO 0 -> RPM Input (via Voltage Divider! 5V -> ~3.3V)
 * - GPIO 1 -> PWM Output (Direct to ESC)
 */

#include <Arduino.h>

// Configuration
#include "config.h"
#include "config_common.h"

// Drivers
#include "pcnt_driver.h"
#include "ledc_driver.h"

// Core Logic
#include "pid_common.h"
#include "rpm_common.h"

// Global Variables
volatile float currentRPM = 0.0;
volatile float targetRPM = DEFAULT_TARGET_RPM;
volatile float pidOutput = 0.0;
volatile int lastPWMValue = 0;

// Filter & PID State
RPMFilter rpmFilter;
float integral = 0.0;
float previousError = 0.0;

// Startup State
bool softStarting = true;
unsigned long softStartStartTime = 0;

// Function Prototypes
void controlLoop(void *parameter);
int applySoftStart(int targetPWM);

void setup() {
    // Initialize Serial for debug (optional, can be removed for production)
    Serial.begin(115200);
    
    // Initialize Drivers
    pcnt_init(RPM_INPUT_PIN);
    ledc_init(PWM_OUTPUT_PIN);
    
    // Create Control Loop Task (High Priority)
    xTaskCreatePinnedToCore(
        controlLoop,    // Function
        "ControlLoop",  // Name
        4096,           // Stack size
        NULL,           // Parameters
        configMAX_PRIORITIES - 1, // Priority (Highest)
        NULL,           // Task handle
        0               // Core ID (0 for ESP32-C3 single core)
    );
    
    Serial.println("ESP32-C3 BLDC Controller Started");
}

void loop() {
    // Main loop is empty, work is done in high-priority task
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void controlLoop(void *parameter) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(CONTROL_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();
    
    while (true) {
        // 1. Calculate RPM
        float rawRPM = pcnt_get_rpm();
        
        // 2. Apply sliding window filter
        float filteredRPM = rpmFilter.update(rawRPM);
        currentRPM = filteredRPM;
        
        // 3. Compute PID
        float error = targetRPM - currentRPM;
        pidOutput = computePID_float(error, integral, previousError, 
                                   DEFAULT_KP, DEFAULT_KI, DEFAULT_KD,
                                   INTEGRAL_WINDUP_MIN, INTEGRAL_WINDUP_MAX,
                                   PID_OUTPUT_MIN, PID_OUTPUT_MAX);
                                   
        // 4. Map to PWM
        int targetPWM = map(pidOutput, PID_OUTPUT_MIN, PID_OUTPUT_MAX, PWM_MIN_VALUE, PWM_MAX_VALUE);
        targetPWM = constrain(targetPWM, PWM_MIN_THRESHOLD, PWM_MAX_VALUE);
        
        // 5. Apply Soft Start (Kickstart)
        int finalPWM = applySoftStart(targetPWM);
        
        // 6. Output to ESC
        motor_set_pwm(finalPWM);
        lastPWMValue = finalPWM;
        
        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

int applySoftStart(int targetPWM) {
    if (!softStarting) return targetPWM;
    
    if (softStartStartTime == 0) softStartStartTime = millis();
    
    unsigned long elapsed = millis() - softStartStartTime;
    if (elapsed >= SOFT_START_DURATION_MS) {
        softStarting = false;
        return targetPWM;
    }
    
    float progress = (float)elapsed / SOFT_START_DURATION_MS;
    int kickstartPWM = PWM_MIN_THRESHOLD + (int)((targetPWM - PWM_MIN_THRESHOLD) * progress);
    return (kickstartPWM > targetPWM) ? targetPWM : kickstartPWM;
}
