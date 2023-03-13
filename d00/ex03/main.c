#include <avr/io.h>
#include <util/delay.h>
#define DEBOUNCE 25 // time in ms
#define LOCK_INPUT 300 // time in ms

int main(void){

    DDRB |= (1 << PB0); // Define led pin (PB0) as output
    DDRD &= (0 << PD2); // Define button pin as input

    PORTB &= (0 << PB0); // Led off
    PORTD &= (1 << PD2); // switch 

    while (1) {
        
        if (!(PIND & (1 << PD2))){ // Button is being pressed
            _delay_ms(DEBOUNCE); // Debounce timer
            if (!(PIND & (1 << PD2))){
                PORTB |= (1 << PB0); // Activate led
            }
        }
        else{
            PORTB &= (0 << PB0); // De-activate led
        }
    }
    return (0);
}
