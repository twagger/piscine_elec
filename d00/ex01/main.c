#include <avr/io.h>

int main(void){
    // Chaque bit du registre DDRB positionné à 0 configure la broche
    // correspondante en entrée. Chaque bit à 1 configure la pin en sortie.

    // On configure le bit correspondant au PB0 a 1
    DDRB |= (1<<PB0);

    // On positionne le pin PORTB a la valeur permettant d'alimenter PB0
    PORTB |= (1<<PB0);
}