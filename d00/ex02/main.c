#include <avr/io.h>
#define WAITER 1600000

int main(void){
    double i;

    DDRB |= (1 << PB0); // Define led pin (PB0) as output
    while (1) {
        PORTB ^= (1 << PB0); // Toogle led pin
        i = 16000;
        while (i) {
            i--;
        }
    }
    return (0);
}
