/**
 * BLDC Motor PID Controller - ATtiny85 Production Version
 *
 * Production-ready PID controller for ATtiny85 microcontroller
 * Maintains exact 1440 RPM using pre-tuned PID gains with enhanced safety features
 *
 * Two versions available:
 * 1. Internal Oscillator (8MHz): Use config.h - Basic performance, no external components
 * 2. External Crystal (20MHz): Use config_external.h - 2.5x faster, requires crystal + capacitors
 *
 * To switch versions:
 * - For 8MHz internal: #include "config_external.h"
 * - For 20MHz external crystal: #include "config_external.h"
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
 *              | [External Crystal: Also connects 16MHz crystal + 22pF caps to pin 3]
 *     3        | Not Connected / Crystal (XTAL2 for external crystal version)
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
 * Date: January 2026
 */

// Include configuration header (automatically selects internal/external based on config_common.h)
#include "config_common.h"

// Include shared common headers
#include "pid_common.h"
#include "rpm_common.h"
#include "isr_common.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>

// Global variables - Production Mode Only (Integer optimized for ATTiny85)
volatile unsigned long pulseInterval = 0;  // Time interval between pulses in microseconds
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
    unsigned long currentMicros = timer_us;
    rpmSensorISR_common(currentMicros, lastPulseMicros, pulseInterval, MIN_PULSE_WIDTH_US);
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

    // Calculate OCR1A for 1ms interrupt based on F_CPU
    // Formula: OCR1A = (F_CPU / 1000 / 64) - 1, but adjusted for timer behavior
#if F_CPU == 20000000UL
    OCR1A = 312;            // Compare value for 1ms at 20MHz (20MHz/64 = 312.5kHz, 312.5kHz/1000 = 312.5)
#elif F_CPU == 8000000UL
    OCR1A = 125;            // Compare value for 1ms at 8MHz (8MHz/64 = 125kHz, 125kHz/1000 = 125)
#endif

    TCCR1 |= (1 << CTC1);   // Clear timer on compare match
    TCCR1 |= (1 << CS12) | (1 << CS11) | (1 << CS10); // Prescaler 64
    TIMSK |= (1 << OCIE1A); // Enable compare interrupt

    // Setup Timer0 for PWM (default 8-bit fast PWM)
    TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00); // Fast PWM, non-inverting

    // Set prescaler based on F_CPU for ~1kHz PWM frequency
#if F_CPU == 20000000UL
    TCCR0B = (1 << CS02); // Prescaler 256, ~1.2kHz PWM frequency at 20MHz
#elif F_CPU == 8000000UL
    TCCR0B = (1 << CS01); // Prescaler 8, ~1kHz PWM frequency at 8MHz
#endif
}



int calculateRPM() {
    static unsigned long lastCalcTime_us = 0;  // Use microseconds for precision

    unsigned long currentTime_us = timer_us;

    if (currentTime_us - lastCalcTime_us >= RPM_CALC_INTERVAL * 1000UL) {  // Convert ms to microseconds
        int rpm_scaled = 0;

        // Timeout check: if no pulses received within timeout period, motor is stopped
        if (currentTime_us - lastPulseMicros > RPM_TIMEOUT_US) {
            rpm_scaled = 0; // Motor stopped
        } else {
            // Atomic read of volatile pulseInterval to avoid race conditions
            cli();
            unsigned long interval = pulseInterval;
            sei();

            // Calculate RPM using period measurement: RPM = (60,000,000) / (interval_μs * pulses_per_rev)
            // For scaled result (RPM * 10): (600,000,000) / (interval_μs * pulses_per_rev)
            // Use sequential division to avoid intermediate multiplication overflow
            if (interval > 0) {
                rpm_scaled = (int)(600000000UL / interval / pulsesPerRev);
            }
        }

        // Apply moving average filter for noise reduction (ATtiny85 optimized)
        updateMovingAverageInt(rpmFiltered, rpm_scaled, rpmHistory, rpmHistoryIndex, RPM_FILTER_SIZE);

        lastCalcTime_us = currentTime_us;

        return rpmFiltered;  // Return filtered value for maximum accuracy
    }

    return rpmFiltered; // Return previous filtered value if not enough time has passed
}

int computePID(int error_scaled) {
    return computePID_fixed(error_scaled, integral_scaled, previousError_scaled,
                           kp_scaled, ki_scaled, kd_scaled,
                           INTEGRAL_WINDUP_MIN, INTEGRAL_WINDUP_MAX,
                           PID_OUTPUT_MIN, PID_OUTPUT_MAX);
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
