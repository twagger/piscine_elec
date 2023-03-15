#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#ifndef F_CPU
# define F_CPU 16000000UL
#endif
#define MAX_VAL 65535
#define DELAY 10

// Variation direction
uint8_t dir = 0;
float   duty_cycle = 0.1;

// Interrupt Service Routine of timer 0 that triggers on compare match
ISR(TIMER0_COMPA_vect) {

    // update duty_cycle
    if (dir == 0 && duty_cycle < 1) {
        duty_cycle += 0.1;
    }
    else if (dir == 0 && duty_cycle >= 1) {
        duty_cycle = 1;
        dir = 1;
    }
    else if (dir == 1 && duty_cycle > 0) {
        duty_cycle -= 0.1;
    }
    else if (dir == 1 && duty_cycle <= 0) {
        duty_cycle = 0;
        dir = 0;
    }
    // Update timer 1 compare value
    OCR1A = MAX_VAL * duty_cycle;
    // Minimun delay between 2 calls to control PB1 duty cycle changes delay
    _delay_ms(DELAY);
}

void    timer_1_conf(float duty_cycle) {
    /*
    This function configures the timer 1 to act on PB1 in a certain way 
    depending on a duty cycle (managed by OCR1A register).
    */

    // 1. Action on OC1A (PB1) on compare match. Here : non-inverting Compare
    // Output mode
    TCCR1A = (1 << COM1A1);
    // 2. Waveform Generation Mode: Fast PWM (Pulse With Modulation) with IRC1
    // as TOP
    TCCR1A &= ~(1 << WGM10);
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM12) | (1 << WGM13);
    // 2bis. Use ICR1 as max value
    ICR1 = MAX_VAL;
    // 3. Value to compare the timer with
    OCR1A = MAX_VAL * duty_cycle;
    // 4. Clock prescale factor (256) + launch the timer
    TCCR1B |= (1 << CS12);
}

void    timer_0_conf() {
    /*
    This function configures the timer 0 to produce an interrupt (INT0) on a
    certain delay that will change the compare value of timer 1 so it changes
    the behaviour of PB1 led indirectly.
    */

    // 1. Action on OC0A (PD6) on compare match. Here : nothing
    TCCR0A &= ~((1 << COM0A0) | (1 << COM0A1));
    // 2. Waveform Generation Mode: CTC (Clear Timer on Compare Match)
    TCCR0A |= (1 << WGM01);
    // 3. Value to compare the timer with (here max val as we just want a 
    // regular delay)
    OCR0A = 251; // This will make the timer 1 tick 31 times per second
    // 4. Interrupts when compare match
    TIMSK0 |= (1 << OCIE0A);
    // 5. Clock prescale factor (1024) + launch the timer
    TCCR0B |= (1 << CS02) | (1 << CS00);
}

int main(void){

    // Set the led D2 (PB1) as output
    DDRB |= (1 << PB1);

    // Init the led state to off
    PORTB &= ~(1 << PB0);

    // Configure the timer 1 to act on PB1
    timer_1_conf(duty_cycle);

    // Enable global interrupt by setting global interrupt enable bit to 1
    SREG |= (1 << 7); // sei(); is also valid but I like the "raw" way

    // Configure timer 0 to trigger an interrupt that will act on timer 1
    timer_0_conf();

    while (1) {

    }

    return (0);
}
