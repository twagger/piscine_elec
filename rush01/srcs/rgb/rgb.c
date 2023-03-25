#include "rgb.h"

/*
** -----------------------------------------------------------------------------
** RGB Functions
** -----------------------------------------------------------------------------
*/
void    init_rgb(void){
    /*
    ** Set the led as output and turn them off
    */

    const uint32_t  rgb[3] = {DEL_RED, DEL_GREEN, DEL_BLUE};
    const uint8_t   nb_rgb = sizeof(rgb) / sizeof(rgb[0]);

    for (uint8_t i = 0; i < nb_rgb; i++) {
        DDRD |= (1 << rgb[i]); // Output
        PORTD &= ~(1 << rgb[i]); // Off by default
    }
}

void    display_rgb(uint32_t rgb){
    /*
    ** Convert a rgb color on 24 bits (FFFFFF) to a led response
    */

    // Set all color off
    PORTD &= ~((1 << DEL_RED) | (1 << DEL_GREEN) | (1 << DEL_BLUE));

    // RED
    if ((rgb & RED) == RED) { PORTD |= (1 << DEL_RED); }

    // GREEN
    if ((rgb & GREEN) == GREEN) { PORTD |= (1 << DEL_GREEN); }

    // BLUE
    if ((rgb & BLUE) == BLUE) { PORTD |= (1 << DEL_BLUE); }

}
