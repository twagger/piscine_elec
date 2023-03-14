#include <avr/io.h>
#ifndef F_CPU
# define F_CPU 16000000UL
#endif

int main(void){

    // Set the PB1 led as output
    DDRB |= (1 << PB1);

    // Configure the timer
    // 1. Toogle the OC1A (PB1) on compare match. This will make the led blink 
    // each time the timer reaches a certain number
    TCCR1A |= (1 << COM1A0);
    // 2. Waveform Generation Mode, 2 = CTC mode : Clear Timer on Compare Match
    // With this mode the timer will count from 00 to OCR1A value (TOP)
    TCCR1B |= (1 << WGM12);
    // 3. Set a value to compare the timer with and reset it when equal
    // The value is calculated from the formula :
    // fOC1A = fclock / (2 * prescaler factor * (1 + OCR1A))
    OCR1A = 31249;
    // 4. Clock prescale factor (256 here) + launch the timer
    TCCR1B |= (1 << CS12);

    return (0);
}
