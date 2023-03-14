#include <avr/io.h>
#include <util/delay.h>
#define DELAY 250
#define SHORT_DELAY 25

int main(void){

    const uint8_t   leds[] = {PB0, PB1, PB2, PB4};
    const uint8_t   nb_leds = sizeof(leds) / sizeof(leds[0]);

    for (uint8_t i = 0; i < nb_leds; i++) {
        DDRB |= (1 << leds[i]); // Outputs
    }
   
    while (1) {
        
        // Loop through the leds in one direction
        for (uint8_t i = 0; i < nb_leds - 1; i++) {
            // Turn off all other lights
            PORTB &= ~(1 << leds[(i + 1) % nb_leds]);
            PORTB &= ~(1 << leds[(i + 2) % nb_leds]);
            PORTB &= ~(1 << leds[(i + 3) % nb_leds]);
            // Turn on the led
            PORTB |= (1 << leds[i]);
            // Wait a little bit
            if (i == 0) {
                _delay_ms(DELAY);
            }
            else { _delay_ms(SHORT_DELAY); }
        }

        // Loop in the other direction    
        for (uint8_t i = nb_leds - 1; i > 0; i--) {
            // Turn off all other lights
            PORTB &= ~(1 << leds[(i + 1) % nb_leds]);
            PORTB &= ~(1 << leds[(i + 2) % nb_leds]);
            PORTB &= ~(1 << leds[(i + 3) % nb_leds]);
            // Turn on the led
            PORTB |= (1 << leds[i]);
            // Wait a little bit
            if (i == nb_leds - 1) {
                _delay_ms(DELAY);
            }
            else { _delay_ms(SHORT_DELAY); }
        }
    }
    return (0);
}
