/*
 * ATtiny85 Production Blink - 8MHz Internal
 * Security: Direct register access to prevent unintended pin states
 * Performance: Optimized bit-toggling via PINB
 */

#ifndef F_CPU
#define F_CPU 8000000UL // Tell the compiler we are at 8MHz
#endif

#include <avr/io.h>
#include <util/delay.h>

int main(void) {
    // Set PB0 (Pin 5) as Output
    // DDRB: Data Direction Register B
    DDRB |= (1 << PB0);

    // Security-First: Ensure all other PORTB pins are tri-stated/low
    // to prevent floating inputs if not used.
    PORTB &= (1 << PB0);

    while (1) {
        // Efficiency: Writing a logic 1 to a bit in PINB
        // toggles the corresponding PORTB bit in 1 clock cycle.
        PINB |= (1 << PB0);

        // Precise delay for 8MHz
        _delay_ms(500);
    }

    return 0;
}
