/**
 * BLDC Motor PID Controller - ESP32-C3 Bluetooth BLE Version
 *
 * BLE control interface
 * Connect via BLE, write commands to control
 *
 * Hardware: Same as control/ (GPIO 0 RPM, GPIO 1 PWM)
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "config.h"
#include "config_common.h"

#include "pcnt_driver.h"
#include "ledc_driver.h"
#include "pid_common.h"
#include "rpm_common.h"

// BLE UUIDs
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define STATUS_CHAR_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CONTROL_CHAR_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define TARGET_CHAR_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26aa"

volatile float currentRPM = 0.0;
volatile float targetRPM = DEFAULT_TARGET_RPM;
volatile float pidOutput = 0.0;
volatile int lastPWMValue = 0;
volatile bool running = false;

RPMFilter rpmFilter;
float integral = 0.0;
float previousError = 0.0;

bool softStarting = true;
unsigned long softStartStartTime = 0;

BLEServer *pServer = NULL;
BLECharacteristic *pStatusChar = NULL;
BLECharacteristic *pControlChar = NULL;
BLECharacteristic *pTargetChar = NULL;
bool deviceConnected = false;

class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        BLEDevice::startAdvertising();
    }
};

class ControlCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String value = pCharacteristic->getValue();
        if (value == "start") {
            running = true;
            softStarting = true;
            softStartStartTime = 0;
            integral = 0.0;
            previousError = 0.0;
            rpmFilter.reset();
        } else if (value == "stop") {
            running = false;
            ledc_set_duty(0);
            lastPWMValue = 0;
        }
    }
};

class TargetCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        uint8_t* data = pCharacteristic->getData();
        size_t len = pCharacteristic->getLength();
        
        if (len > 0) {
            char buffer[16];
            size_t copyLen = (len < 15) ? len : 15;
            memcpy(buffer, data, copyLen);
            buffer[copyLen] = '\0';
            
            float rpm = atof(buffer);
            if (rpm >= TARGET_RPM_MIN && rpm <= TARGET_RPM_MAX) {
                targetRPM = rpm;
                
                char response[32];
                snprintf(response, sizeof(response), "%.0f", targetRPM);
                pCharacteristic->setValue(response);
            }
        }
    }
};

void controlLoop(void* parameter);
int applySoftStart(int targetPWM);
void setupBLE();
void updateStatus();

void setup() {
    pcnt_init(RPM_INPUT_PIN);
    ledc_init(PWM_OUTPUT_PIN);

    setupBLE();

    xTaskCreatePinnedToCore(
        controlLoop,
        "ControlLoop",
        4096,
        NULL,
        configMAX_PRIORITIES - 1,
        NULL,
        0
    );
}

void loop() {
    if (deviceConnected) {
        updateStatus();
    }
    delay(500);
}

void setupBLE() {
    BLEDevice::init("BLDC-PID");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    pStatusChar = pService->createCharacteristic(
        STATUS_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pStatusChar->addDescriptor(new BLE2902());

    pControlChar = pService->createCharacteristic(
        CONTROL_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pControlChar->setCallbacks(new ControlCallbacks());

    pTargetChar = pService->createCharacteristic(
        TARGET_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ
    );
    pTargetChar->setCallbacks(new TargetCallbacks());

    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();
}

void updateStatus() {
    char status[128];
    snprintf(status, sizeof(status), "{\"run\":%d,\"rpm\":%.0f,\"target\":%.0f,\"pwm\":%d}",
        running ? 1 : 0, currentRPM, targetRPM, lastPWMValue);
    pStatusChar->setValue(status);
    pStatusChar->notify();
}

void controlLoop(void* parameter) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(CONTROL_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();

    while (true) {
        float rawRPM = pcnt_get_rpm();
        
        // Apply sliding window filter
        currentRPM = rpmFilter.update(rawRPM);

        if (!running) {
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }

        float error = targetRPM - currentRPM;
        pidOutput = computePID_float(error, integral, previousError,
            DEFAULT_KP, DEFAULT_KI, DEFAULT_KD,
            INTEGRAL_WINDUP_MIN, INTEGRAL_WINDUP_MAX,
            PID_OUTPUT_MIN, PID_OUTPUT_MAX);

        int targetPWM = map(pidOutput, PID_OUTPUT_MIN, PID_OUTPUT_MAX, PWM_MIN_VALUE, PWM_MAX_VALUE);
        targetPWM = constrain(targetPWM, PWM_MIN_THRESHOLD, PWM_MAX_VALUE);

        int finalPWM = applySoftStart(targetPWM);
        ledc_set_duty(finalPWM);
        lastPWMValue = finalPWM;

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
