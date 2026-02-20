/*
 * ATtiny85 Closed-Loop Motor Control - 8MHz Internal
 * Security: Minimal ISR logic to prevent race conditions
 * Performance: Hardware PWM (Timer0) + Precision Timer1 Capture
 *
 * Target: 1440 RPM
 * Sensor: 4 PPR (Pulses Per Revolution)
 * Input: PB3 (Physical Pin 2) - Hall Sensor
 * Output: PB0 (Physical Pin 5) - PWM Motor Driver
 * Clock: 16MHz Internal PLL (Requires fuse change)
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// --- Configuration ---
#define TARGET_RPM 1440
#define SENSOR_PPR 4
// Calculate target frequency: (1440 / 60) * 4 = 96 Hz
// Calculate target period in timer ticks:
// Timer1 Clock = 16MHz / 256 = 62500 Hz (16us per tick)
// Target Period (ms) = 1000 / 96 = 10.416 ms
// Target Ticks = 10.416ms / 0.016ms = 651 ticks
#define TARGET_TICKS 651

// PID Gains (tuned for typical DC motor response)
// Scaled by 100 for integer math
#define K_P 400  // Increased Gain: Error of 1 tick -> 400 output
#define K_I 20   // Integral Gain

// --- Global Variables ---
volatile uint16_t timer1_overflows = 0;
volatile uint32_t last_capture_time = 0;
volatile uint32_t pulse_period = 0;
volatile uint8_t new_pulse = 0;
uint32_t period_history[6] = {0, 0, 0, 0, 0, 0}; // Moving average buffer (6 samples)
uint8_t history_idx = 0;

// --- Interrupt Service Routines ---

// Timer1 Overflow Interrupt
// Increments the high-word of our time counter
ISR(TIMER1_OVF_vect) {
    timer1_overflows++;
}

// Pin Change Interrupt (PB3)
// Captures the exact time of the sensor pulse
ISR(PCINT0_vect) {
    // Check if PB3 is HIGH (Rising Edge)
    if (PINB & (1 << PB3)) {
        // Atomic Reading of Time
        uint8_t low_byte = TCNT1;
        uint16_t high_word = timer1_overflows;

        // Critical: Check for pending overflow that happened during ISR entry
        // If TOV1 is set and TCNT1 is small, the overflow happened but ISR hasn't run yet
        if ((TIFR & (1 << TOV1)) && (low_byte < 128)) {
            high_word++;
        }

        uint32_t current_time = ((uint32_t)high_word << 8) | low_byte;
        
        // Calculate period (handle wrap-around naturally with unsigned math)
        pulse_period = current_time - last_capture_time;
        last_capture_time = current_time;
        
        new_pulse = 1;
    }
}

// --- Setup Functions ---

void setup_pwm() {
    // Timer0: Fast PWM on PB0
    DDRB |= (1 << PB0); // Output
    TCCR0A = (1 << WGM00) | (1 << WGM01) | (1 << COM0A1);
    TCCR0B = (1 << CS01); // Prescaler /8
    // PWM Freq = 16MHz / (256 * 8) = 7.8kHz
}

void setup_timer1() {
    // Timer1: Normal Mode, Prescaler /256
    // 16MHz / 256 = 62.5 kHz (16us per tick)
    // Overflow every 256 * 16us = 4.096 ms
    TCCR1 = (1 << CS13) | (1 << CS10); // CS13=1, CS10=1 -> /256
    TIMSK |= (1 << TOIE1); // Enable Overflow Interrupt
}

void setup_pcint() {
    // Enable Pin Change Interrupt on PB3
    DDRB &= ~(1 << PB3); // Input
    PORTB |= (1 << PB3); // Pull-up (Hall sensors usually Open-Drain)
    GIMSK |= (1 << PCIE); // Enable PC Interrupts
    PCMSK |= (1 << PCINT3); // Mask for PB3
}

int main(void) {
    setup_pwm();
    setup_timer1();
    setup_pcint();
    
    // Enable Global Interrupts early to monitor speed during Soft-Start
    sei();
    
    // --- Soft Start Sequence ---
    // Ramp up from 25% (64) to 50% (128) over 1.5 seconds
    // This overcomes static friction without violent jerking
    uint8_t start_duty = 64;
    OCR0A = start_duty;
    
    int32_t integral = 0;
    
    for (int i = 0; i < 150; i++) {
        _delay_ms(10); // 10ms * 150 = 1.5s
        
        // Safety Check: If we reached target speed (period <= target), stop ramping!
        // TARGET_TICKS = 651. Lower ticks = Higher Speed.
        // If period is valid (>0) and less than target, we are fast enough.
        if (new_pulse && pulse_period > 0 && pulse_period <= TARGET_TICKS) {
            // Pre-load integral so the transition to PID is smooth.
            // PID Equation: Final = 128 + (P + I)/100
            // We want Final = start_duty.
            // Assume Error is approx 0 (at target).
            // start_duty = 128 + I/100  ->  I = (start_duty - 128) * 100
            // Since start_duty <= 128, integral will be negative or zero.
            integral = ((int32_t)start_duty - 128) * 100 / K_I;
            break; 
        }
        
        // Linear ramp up
        if (start_duty < 128) {
            start_duty++;
            OCR0A = start_duty;
        }
    }
    
    // Clear any accumulated flags but KEEP the period history for smooth PID start
    new_pulse = 0;
    
    int32_t error = 0;

    while (1) {
        if (new_pulse) {
            new_pulse = 0; // Clear flag

            // --- Moving Average Filter ---
            // Smooth out sensor jitter
            period_history[history_idx++] = pulse_period;
            if (history_idx >= 6) history_idx = 0;
            
            uint32_t avg_period = 0;
            for (uint8_t i = 0; i < 6; i++) avg_period += period_history[i];
            avg_period /= 6;

            // --- PID Control Loop ---
            
            // Calculate Error (Target - Measured)
            // Positive Error = Too Slow (Period is larger than Target) -> Increase Power
            // Use averaged period for stability
            int32_t error = (int32_t)avg_period - TARGET_TICKS;

            // Integral Term (Accumulate error)
            integral += error;
            
            // Anti-Windup for Integral
            // Map Range: -5000 to +5000 (Internal PID Scale)
            if (integral > 500000) integral = 500000;
            if (integral < -500000) integral = -500000;

            // Calculate PI Output (Raw Scale)
            // Error is typically +/- 50 ticks
            // P_Term = 50 * 400 = 20,000
            int32_t p_term = error * K_P;
            int32_t i_term = integral * K_I;
            
            int32_t pid_sum = (p_term + i_term) / 100;

            // Map PID Output (-5000 to +5000) to PWM (45 to 255)
            // Center point (0 error) -> 150 PWM (Approx 60%)
            // Range +/- 5000 maps to +/- 105 PWM steps
            // Slope = 105 / 5000 = 0.021
            // Output = 150 + (pid_sum / 48)
            
            int16_t mapped_output = 150 + (int16_t)(pid_sum / 48);

            // Clamp Output
            if (mapped_output > 255) mapped_output = 255;
            if (mapped_output < 45) mapped_output = 45; // Min Power 45 (Stall prevention)

            // Update PWM
            OCR0A = (uint8_t)mapped_output;
        }
    }

    return 0;
}
