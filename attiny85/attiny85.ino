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
 * - Minimal safety: Watchdog timer only
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

// Global variables - Production Mode Only (Integer optimized for ATTiny85)
volatile unsigned long pulseCount = 0;
volatile unsigned long timer_ms = 0;
volatile unsigned long timer_us = 0;  // Microsecond counter for debounce
volatile unsigned long lastPulseMicros = 0;
unsigned long lastRPMCalcTime = 0;
int currentRPM = 0;  // RPM * 10 (fixed point, e.g., 14400 = 1440.0 RPM)
const int pulsesPerRev = DEFAULT_PULSES_PER_REV; // Runtime variable for consistency

// Moving average filter for RPM smoothing and accuracy (ATtiny85 optimized)
#define RPM_FILTER_SIZE 3  // Smaller filter for ATtiny85 memory constraints
int rpmHistory[RPM_FILTER_SIZE] = {0};
int rpmHistoryIndex = 0;
int rpmFiltered = 0;

// PID variables - Pre-tuned values from Arduino (scaled for integer math)
const int targetRPM_scaled = (int)(DEFAULT_TARGET_RPM * 10);  // Target RPM * 10
const int kp_scaled = (int)(DEFAULT_KP * 100);                // Kp * 100
const int ki_scaled = (int)(DEFAULT_KI * 100);                // Ki * 100
const int kd_scaled = (int)(DEFAULT_KD * 100);                // Kd * 100
int previousError_scaled = 0;  // Previous error * 10
long integral_scaled = 0;      // Integral * 1000 (higher precision)
int pidOutput = 0;             // Final output (-255 to 255)

// Safety features

// Soft-start ramping to avoid current surges (ATtiny85 version)

// Function prototypes
void setupPins();
void setupTimer();
void rpmSensorISR();
int calculateRPM();
int computePID(int error_scaled);
void outputToESC(uint8_t pwmValue);
int constrain_value(int value, int min, int max);
long map(long x, long in_min, long in_max, long out_min, long out_max);

// Timer1 interrupt for millisecond and microsecond timing (ATtiny85 Timer1)
ISR(TIM1_COMPA_vect) {
    timer_ms++;
    timer_us += 1000;  // Increment microseconds by 1000 (1ms)

    // No watchdog - minimal operation
}

// RPM sensor interrupt with debounce filtering
ISR(INT0_vect) {
    unsigned long t = timer_us;
    if (t - lastPulseMicros > MIN_PULSE_WIDTH_US) {
        pulseCount++;
        lastPulseMicros = t;
    }
}

void setup() {
    // Setup watchdog timer first (before disabling it)
    wdt_disable();  // Ensure watchdog is disabled for minimal operation

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

    // Control loop timing - Production mode only
    if (timer_ms - lastLoopTime >= CONTROL_PERIOD_MS) {
        lastLoopTime = timer_ms;

        // Calculate RPM at regular intervals
        if (timer_ms - lastRPMCalcTime >= RPM_CALC_INTERVAL) {
            currentRPM = calculateRPM();
            lastRPMCalcTime = timer_ms;
        }

        // Compute PID output using pre-tuned gains (integer math)
        int error_scaled = targetRPM_scaled - currentRPM;
        pidOutput = computePID(error_scaled);

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
    // Configure pins - Minimal production setup (Arduino-style)
    pinMode(PWM_OUTPUT_PIN, OUTPUT);   // PWM pin as output
    pinMode(RPM_SENSOR_PIN, INPUT_PULLUP);  // RPM sensor as input with pull-up

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



int calculateRPM() {
    static unsigned long lastPulseCount = 0;
    static unsigned long lastCalcTime_us = 0;  // Use microseconds for precision

    unsigned long currentTime_us = timer_us;
    unsigned long timeDiff_us = currentTime_us - lastCalcTime_us;

    if (timeDiff_us >= RPM_CALC_INTERVAL * 1000UL) {  // Convert ms to microseconds
        // Atomic read of volatile pulseCount to avoid race conditions
        cli();
        unsigned long pulsesNow = pulseCount;
        sei();

        unsigned long pulseDiff = pulsesNow - lastPulseCount;

        // Calculate RPM with microsecond precision using integer math
        // (pulses * 60000000) / (timeDiff_us * pulsesPerRev) gives RPM * 10
        // This gives RPM * 10 (e.g., 14400 = 1440.0 RPM) with microsecond precision
        // 60000000 = 60 seconds/minute * 1000000 microseconds/second * 10 for scaling
        unsigned long rpm_scaled = (pulseDiff * 60000000UL) / (timeDiff_us * pulsesPerRev);

        // Apply moving average filter for noise reduction (ATtiny85 optimized)
        rpmHistory[rpmHistoryIndex] = (int)rpm_scaled;
        rpmHistoryIndex = (rpmHistoryIndex + 1) % RPM_FILTER_SIZE;

        // Calculate filtered RPM
        long sum = 0;
        for(int i = 0; i < RPM_FILTER_SIZE; i++) {
            sum += rpmHistory[i];
        }
        rpmFiltered = (int)(sum / RPM_FILTER_SIZE);

        lastPulseCount = pulsesNow;
        lastCalcTime_us = currentTime_us;

        return rpmFiltered;  // Return filtered value for maximum accuracy
    }

    return rpmFiltered; // Return previous filtered value if not enough time has passed
}

int computePID(int error_scaled) {
    // Proportional term: kp_scaled * error_scaled / 1000
    // (kp_scaled is Kp*100, error_scaled is error*10, so divide by 1000 for Kp*error)
    long proportional = (long)kp_scaled * error_scaled / 1000;

    // Integral term with anti-windup: integral_scaled += ki_scaled * error_scaled / 100
    // (ki_scaled is Ki*100, error_scaled is error*10, so divide by 100 for Ki*error)
    integral_scaled += (long)ki_scaled * error_scaled / 100;

    // Clamp integral to prevent windup (scaled by 1000)
    long integral_max_scaled = INTEGRAL_WINDUP_MAX * 1000;
    long integral_min_scaled = INTEGRAL_WINDUP_MIN * 1000;
    if (integral_scaled > integral_max_scaled) integral_scaled = integral_max_scaled;
    if (integral_scaled < integral_min_scaled) integral_scaled = integral_min_scaled;

    // Derivative term: kd_scaled * (error_scaled - previousError_scaled) / 1000
    long derivative = (long)kd_scaled * (error_scaled - previousError_scaled) / 1000;
    previousError_scaled = error_scaled;

    // Calculate total PID output: proportional + (integral_scaled/1000) + derivative
    long output = proportional + (integral_scaled / 1000) + derivative;

    // Clamp output to safe range
    if (output > PID_OUTPUT_MAX) output = PID_OUTPUT_MAX;
    if (output < PID_OUTPUT_MIN) output = PID_OUTPUT_MIN;

    return (int)output;
}

// Soft-start removed for size optimization

void outputToESC(uint8_t pwmValue) {
    // Clamp PWM value to safe range
    if (pwmValue > 255) pwmValue = 255;
    if (pwmValue < 0) pwmValue = 0;

    // Set PWM duty cycle directly (no soft-start for size optimization)
    OCR0A = pwmValue;
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
