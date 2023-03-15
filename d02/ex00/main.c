#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#ifndef F_CPU
# define F_CPU 16000000UL
#endif
#define DEBOUNCING 250 // debouncing delay (in ms)

// Interrupt Service Routine
ISR(INT0_vect) {
    PORTB ^= (1 << PB0);
    _delay_ms(DEBOUNCING);
    EIFR = (1 << INTF0); // clear external interrupt flag 
} 

int main(void){

    // Define led D1 (PB0) as output
    DDRB |= (1 << PB0);

    // Init the led state to off
    PORTB &= ~(1 << PB0);

    // Set the Switch as input to enable pull up resistor
    // (It helps a little bit with bouncing)
    PORTD |= (1 << PD2);

    // Parameterization of the interrupt
    // 1. Sense control : low level
    // As bouncing will generates more edges than level, it is safer to chose
    // the low level instead of the falling or rising edge events.
    // EICRA &= ~((1 << ISC00) | (1 << ISC01));
    EICRA &= ~(1 << ISC00);
    EICRA |= (1 << ISC01);
    // 2. Enable INT0 interrupt
    EIMSK |= (1 << INT0);
    // // 3. Enable global interrupt by setting global interrupt enable bit to 1
    SREG |= (1 << 7); // sei(); is also valid

    while (1) {

    }

    return (0);
}
