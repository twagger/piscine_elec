#include <avr/io.h>
#include <util/delay.h>
#define DEBOUNCE 25 // time in ms
#define LOCK_INPUT 300 // time in ms

char    is_pressed(int pinx, int btn){
    if (!(pinx & (1 << btn))){
        _delay_ms(DEBOUNCE);
        if (!(pinx & (1 << btn))){
            return (1);
        }
    }
    return (0);
}

int main(void){

    const uint8_t   leds[] = {PB0, PB1, PB2, PB4};
    const uint8_t   switchs[] = {PD2, PD4};
    const uint8_t   nb_leds = sizeof(leds) / sizeof(leds[0]);
    const uint8_t   nb_switchs = sizeof(switchs) / sizeof(switchs[0]);
    uint8_t         sum;

    // Init    
    for (uint8_t i = 0; i < nb_leds; i++) {
        DDRB |= (1 << leds[i]); // Outputs
        PORTB &= ~(1 << leds[i]); // Init
    }
    for (uint8_t i = 0; i < nb_switchs; i++) {
        DDRD &= ~(1 << switchs[i]); // Inputs
        PORTD |= (1 << switchs[i]); // Helps a little bit with bouncing
    }

    sum = 0;
    while (1) {
        
        // Check switches
        for (uint8_t i = 0; i < nb_switchs; i++) {
            if (is_pressed(PIND, switchs[i])){ // Button is being pressed
                sum = (i == 0) ? sum + 1 : sum - 1; // Update sum
                _delay_ms(LOCK_INPUT); // Lock delay to avoid side effects
            }
        }

        // Update leds with the binary version of the sum
        if (sum < 16 || sum > 0){
            for (uint8_t i = 0; i < nb_leds; i++) {
                if (sum & 1 << i){
                    PORTB |= (1 << leds[i]);
                }
                else { PORTB &= ~(1 << leds[i]); }
            }
        }

    }
    return (0);
}
