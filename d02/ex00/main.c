#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#ifndef F_CPU
# define F_CPU 16000000UL
#endif
#define LOCK_INPUT 300 // time in ms

// Interrupt Service Routine
ISR(INT0_vect) {
    PORTB ^= (1 << PB0);
    // Lock delay to avoid bouncing
    _delay_ms(LOCK_INPUT);
} 

int main(void){

    // Define led D1 (PB0) as output
    DDRB |= (1 << PB0);

    // Init the led state to off
    PORTB &= ~(1 << PB0);

    // Parameterization of the interrupt
    // 1. Sense control : The low level of INT0 generate interrupt request
    EICRA &= ~((1 << ISC01) | (1 << ISC00));
    // 2. Enable INT0 interrupt
    EIMSK |= (1 << INT0);
    // 3. Enable global interrupt by setting global interrupt enable bit to 1
    SREG |= (1 << 7); // sei(); is also valid

    while (1) {

    }

    return (0);
}
