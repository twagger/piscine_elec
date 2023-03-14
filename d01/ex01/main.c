#include <avr/io.h>
#ifndef F_CPU
# define F_CPU 16000000UL
#endif
#define MAX_VAL 65535

float duty_cycle = 0.1;

int main(void){

    // Same as before but the led should be on 10% of the time and off 90% on 
    // each cycle.

    // Set the PB1 led as output
    DDRB |= (1 << PB1);

    // Configure the timer
    // 1. Action on OC1A (PB1) on compare match. Here : non-inverting Compare
    // Output mode
    TCCR1A = (1 << COM1A1);
    // 2. Waveform Generation Mode: Fast PWM (Pulse With Modulation) with IRC1
    // as TOP
    TCCR1A &= ~(1 << WGM10);
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM12) | (1 << WGM13);
    // Use ICR1 as max value
    ICR1 = MAX_VAL;
    // 3. Value to compare the timer with
    OCR1A = MAX_VAL * duty_cycle;
    // 4. Clock prescale factor (256) + launch the timer
    TCCR1B |= (1 << CS12);

    while (1) {

    }

    return (0);
}
