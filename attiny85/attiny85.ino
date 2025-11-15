/**
 * BLDC Motor PID Controller - ATtiny85 Production Version
 *
 * Production-ready PID controller for ATtiny85 microcontroller
 * Maintains exact 1440 RPM using pre-tuned PID gains with enhanced safety features
 *
 * Motor Compatibility:
 * - Designed for 3-Hall BLDC motors (such as 42BLF20-22.0223)
 * - Works with any BLDC motor that has 3 built-in Hall effect sensors
 * - Hall sensors provide 6 pulses per electrical revolution
 * - Optimized for production deployment with minimal hardware
 *
 * Hardware Connections (Minimal):
 * Physical Pin | Function
 *     1        | VCC (Power - 5V for Hall sensor compatibility)
 *     2        | BLDC Hall Sensor (any Hall wire A/B/C from motor, interrupt input)
 *     3        | Not Connected
 *     4        | GND (Ground - common with motor Hall sensors)
 *     5        | PWM to ESC (motor control output)
 *     6        | Not Connected
 *     7        | Not Connected
 *     8        | Not Connected
 *
 * Production Features:
 * - No potentiometers (uses hardcoded tuned values)
 * - No mode switch (always production mode)
 * - Minimal pin usage for maximum reliability
 * - Optimized for size and power efficiency
 * - Enhanced safety: Watchdog timer, emergency stop, soft-start
 *
 * Tuned on Arduino, deployed on ATtiny85 for production use
 *
 * Author: azzar budiyanto
 * Co-Author: azzar persona (AI assistant)
 * Date: November 2025
 */

// Include configuration header
#include "config.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>

// Global variables - Production Mode Only
volatile unsigned long pulseCount = 0;
volatile unsigned long timer_ms = 0;
volatile unsigned long timer_us = 0;  // Microsecond counter for debounce
volatile unsigned long lastPulseMicros = 0;
unsigned long lastRPMCalcTime = 0;
float currentRPM = 0.0;
const int pulsesPerRev = PULSES_PER_REV; // Runtime variable for consistency

// PID variables - Pre-tuned values from Arduino
const float targetRPM = PRODUCTION_TARGET_RPM;
const float kp = PRODUCTION_KP;
const float ki = PRODUCTION_KI;
const float kd = PRODUCTION_KD;
float previousError = 0.0;
float integral = 0.0;
float pidOutput = 0.0;

// Safety features
volatile bool emergencyStop = false;
volatile bool motorRunning = false;
volatile unsigned long lastPulseMillis = 0;

// Soft-start ramping to avoid current surges (ATtiny85 version)
volatile bool softStarting = true;
volatile unsigned long softStartStartTime = 0;

// Function prototypes
void setupPins();
void setupTimer();
void setupWatchdog();
void rpmSensorISR();
void watchdogISR();
float calculateRPM();
float computePID(float error);
void outputToESC(uint8_t pwmValue);
void checkSafetyConditions();
void emergencyShutdown();
int constrain_value(int value, int min, int max);
long map(long x, long in_min, long in_max, long out_min, long out_max);

// Timer1 interrupt for millisecond and microsecond timing (ATtiny85 Timer1)
ISR(TIM1_COMPA_vect) {
    timer_ms++;
    timer_us += 1000;  // Increment microseconds by 1000 (1ms)

    // Feed watchdog timer periodically
    if (WATCHDOG_ENABLED) {
        wdt_reset();
    }
}

// Watchdog timer interrupt (called before reset)
ISR(WDT_vect) {
    // Emergency shutdown on watchdog timeout
    emergencyShutdown();
}

// RPM sensor interrupt with debounce filtering
ISR(INT0_vect) {
    unsigned long t = timer_us;
    if (t - lastPulseMicros > MIN_PULSE_WIDTH_US) {
        pulseCount++;
        lastPulseMicros = t;
        lastPulseMillis = timer_ms;  // Track last pulse time for safety
        motorRunning = true;         // Motor is actively producing pulses
    }
}

void setup() {
    // Setup watchdog timer first (before disabling it)
    if (WATCHDOG_ENABLED) {
        setupWatchdog();
    } else {
        wdt_disable();
    }

    setupPins();
    setupTimer();

    // Enable global interrupts
    sei();

    // Brief startup delay
    _delay_ms(1000);

    // Initialize PWM to stopped position
    outputToESC(0);
}

void loop() {
    static unsigned long lastLoopTime = 0;
    static unsigned long lastSafetyCheck = 0;

    // Feed watchdog at start of main loop
    if (WATCHDOG_ENABLED) {
        wdt_reset();
    }

    // Control loop timing - Production mode only
    if (timer_ms - lastLoopTime >= CONTROL_PERIOD_MS) {
        lastLoopTime = timer_ms;

        // Safety checks (run less frequently than main control loop)
        if (EMERGENCY_STOP_ENABLED && timer_ms - lastSafetyCheck >= 250) { // Every 250ms
            checkSafetyConditions();
            lastSafetyCheck = timer_ms;
        }

        // Skip PID control if emergency stop is active
        if (emergencyStop) {
            outputToESC(0);  // Emergency stop: zero PWM
            return;  // Exit loop early for emergency stop
        }

        // Calculate RPM at regular intervals
        if (timer_ms - lastRPMCalcTime >= RPM_CALC_INTERVAL) {
            currentRPM = calculateRPM();
            lastRPMCalcTime = timer_ms;
        }

        // Compute PID output using pre-tuned gains
        float error = targetRPM - currentRPM;
        pidOutput = computePID(error);

        // Convert PID output to PWM value and output to ESC
        int pwmValue = map(pidOutput, PID_OUTPUT_MIN, PID_OUTPUT_MAX, 0, 255);
        pwmValue = constrain_value(pwmValue, 0, 255);

        // Normal operation - control motor
        outputToESC(pwmValue);
    }

    // Small delay to prevent tight polling
    _delay_ms(1);
}

void setupPins() {
    // Configure pins - Minimal production setup
    DDRB |= (1 << PWM_OUTPUT_PIN);     // PWM pin as output
    DDRB &= ~(1 << RPM_SENSOR_PIN);    // RPM sensor as input
    PORTB |= (1 << RPM_SENSOR_PIN);    // Enable pull-up on RPM sensor

    // Configure external interrupt for RPM sensor
    MCUCR |= (1 << ISC01);  // Falling edge trigger
    GIMSK |= (1 << INT0);   // Enable INT0 interrupt
}

void setupTimer() {
    // Setup Timer1 for millisecond timing (ATtiny85 Timer1)
    TCCR1 = 0;              // Stop timer
    TCNT1 = 0;              // Reset counter
    OCR1A = 125;            // Compare value for 1ms at 8MHz (8MHz/64 = 125kHz, 125kHz/1000 = 125)
    TCCR1 |= (1 << CTC1);   // Clear timer on compare match
    TCCR1 |= (1 << CS12) | (1 << CS11) | (1 << CS10); // Prescaler 64
    TIMSK |= (1 << OCIE1A); // Enable compare interrupt

    // Setup Timer0 for PWM (default 8-bit fast PWM)
    TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00); // Fast PWM, non-inverting
    TCCR0B = (1 << CS01); // Prescaler 8, ~1kHz PWM frequency
}

void setupWatchdog() {
    // Configure watchdog timer with interrupt mode (not reset)
    wdt_reset();
    WDTCR |= (1 << WDCE) | (1 << WDE);  // Enable configuration change
    WDTCR = (1 << WDIE) | WATCHDOG_TIMEOUT;  // Interrupt mode, set timeout
}

void checkSafetyConditions() {
    if (!EMERGENCY_STOP_ENABLED) return;

    // Emergency stop condition 1: No pulses received for timeout period
    if (motorRunning && (timer_ms - lastPulseMillis > EMERGENCY_STOP_TIMEOUT_MS)) {
        emergencyStop = true;
        return;
    }

    // Emergency stop condition 2: Motor running too fast (potential runaway)
    if (currentRPM > targetRPM * 2.0) {  // More than 2x target RPM
        emergencyStop = true;
        return;
    }

    // Emergency stop condition 3: Motor not spinning when it should be
    if (pidOutput > 50 && currentRPM < 10.0) {  // High PWM but no RPM
        emergencyStop = true;
        return;
    }

    // Reset emergency stop if conditions are normal
    emergencyStop = false;
}

void emergencyShutdown() {
    // Force PWM to zero in emergency
    OCR0A = 0;

    // Disable interrupts to prevent further operation
    cli();

    // Infinite loop - requires external reset
    while (1) {
        _delay_ms(100);
    }
}

float calculateRPM() {
    static unsigned long lastPulseCount = 0;
    static unsigned long lastCalcTime = 0;

    unsigned long timeDiff = timer_ms - lastCalcTime;

    if (timeDiff >= RPM_CALC_INTERVAL) {
        // Atomic read of volatile pulseCount to avoid race conditions
        cli();
        unsigned long pulsesNow = pulseCount;
        sei();

        unsigned long pulseDiff = pulsesNow - lastPulseCount;

        // Calculate RPM: (pulses / time) * (60 seconds / pulses_per_rev)
        float rpm = (pulseDiff * 60000.0) / (timeDiff * pulsesPerRev);

        lastPulseCount = pulsesNow;
        lastCalcTime = timer_ms;

        return rpm;
    }

    return currentRPM; // Return previous value if not enough time has passed
}

float computePID(float error) {
    // Proportional term
    float proportional = kp * error;

    // Integral term with anti-windup
    integral += ki * error;

    // Clamp integral to prevent windup
    if (integral > INTEGRAL_WINDUP_MAX) integral = INTEGRAL_WINDUP_MAX;
    if (integral < INTEGRAL_WINDUP_MIN) integral = INTEGRAL_WINDUP_MIN;

    // Derivative term
    float derivative = kd * (error - previousError);
    previousError = error;

    // Calculate total PID output
    float output = proportional + integral + derivative;

    // Clamp output to safe range
    if (output > PID_OUTPUT_MAX) output = PID_OUTPUT_MAX;
    if (output < PID_OUTPUT_MIN) output = PID_OUTPUT_MIN;

    return output;
}

// Apply soft-start ramping to avoid current surges (ATtiny85 version)
uint8_t applySoftStart(uint8_t targetPWM) {
    if (!softStarting) {
        return targetPWM;  // Normal operation
    }

    unsigned long currentTime = timer_ms;

    if (softStartStartTime == 0) {
        softStartStartTime = currentTime;
    }

    unsigned long elapsed = currentTime - softStartStartTime;

    if (elapsed >= SOFT_START_DURATION_MS) {
        // Soft-start complete
        softStarting = false;
        return targetPWM;
    }

    // Simple linear ramp for ATtiny85 (avoid floating point)
    unsigned long rampProgress = (elapsed * 255) / SOFT_START_DURATION_MS;
    return (uint8_t)((targetPWM * rampProgress) / 255);
}

void outputToESC(uint8_t pwmValue) {
    uint8_t safePWM = applySoftStart(pwmValue);
    OCR0A = safePWM; // Set PWM duty cycle with soft-start protection
}

// Simple constrain function for ATtiny85
int constrain_value(int value, int min, int max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// Simple map function for ATtiny85
long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
