#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#ifndef F_CPU
# define F_CPU 16000000UL
#endif
#define LOCK_INPUT 25 // time in ms

// globals
uint8_t         sum = 0;
const uint8_t   leds[] = {PB0, PB1, PB2, PB4};
const uint8_t   switchs[] = {PD2, PD4};
const uint8_t   nb_leds = sizeof(leds) / sizeof(leds[0]);
const uint8_t   nb_switchs = sizeof(switchs) / sizeof(switchs[0]);

// Helper functions
void    update_leds() {
    /*
    ** Update leds with the binary version of the sum
    */

    if (sum < 16 || sum > 0){
        for (uint8_t i = 0; i < nb_leds; i++) {
            if (sum & 1 << i){
                PORTB |= (1 << leds[i]);
            }
            else { PORTB &= ~(1 << leds[i]); }
        }
    }
}

// Interrupt Service Routines
ISR(INT0_vect) {
    /*
    ** This function will increment a sum when it is triggered
    */

    ++sum;
    // update leds
    update_leds();
    // Lock delay to avoid bouncing
    _delay_ms(LOCK_INPUT);
}

ISR(PCINT2_vect) {
    /*
    ** This function will decrement a sum when it is triggered
    */

    if ((PIND & (1 << PD4)) == 0) {
        // HIGH to LOW pin change
        --sum;
        // update leds
        update_leds();
        // Lock delay to avoid bouncing
        _delay_ms(LOCK_INPUT);
    }
}

void    int_0_conf() {
    /*
    ** Parameterization of the INT0 interrupt
    */

    // 1. Sense control : The low level of INT0 generate interrupt request
    EICRA &= ~((1 << ISC01) | (1 << ISC00));
    // 2. Enable INT0 interrupt
    EIMSK |= (1 << INT0);
}

void    pcint_20_conf() {
    /*
    ** Parameterization of the PCINT20 interrupt
    ** Carefull : ANY change will trigger the associated ISR
    */

    // Enable PCINT23:16 range interrupts    
    PCICR |= (1 << PCIE2);
    // Enable PCINT20 interrupts only
    PCMSK2 = (1 << PCINT20);
}

int main(void){
    /*
    ** Main program
    */

    // Init    
    for (uint8_t i = 0; i < nb_leds; i++) {
        DDRB |= (1 << leds[i]); // Outputs
        PORTB &= ~(1 << leds[i]); // Init
    }
    for (uint8_t i = 0; i < nb_switchs; i++) {
        DDRD &= ~(1 << switchs[i]); // Inputs
        PORTD &= (1 << switchs[i]);
    }

    // Enable global interrupt by setting global interrupt enable bit to 1
    SREG |= (1 << 7); // sei(); is also valid but I like the "raw" way

    // Configure INT0 interrupt that controls SW1
    int_0_conf();

    // Configure PCINT20 interrupt that controls SW2
    pcint_20_conf();

    while (1) {

    }

    return (0);
}
