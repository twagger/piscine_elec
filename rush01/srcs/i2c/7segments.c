#include "i2c.h"

// 7 segments
const uint8_t   brightness[] = {30, 10}; // ratio on / off on the 7seg
const uint8_t   numbers[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, \
                             0x7F, 0x6F};

/*
** -----------------------------------------------------------------------------
** I2C 7 Segments
** -----------------------------------------------------------------------------
*/
void    clear_all_digits(void){
    /*
    ** Clear 7seg
    */

    // Start
    i2c_start(EXPANDER_W, TW_MT_SLA_ACK);
    
    // Write to output port 0 and 1
    i2c_write(0x02);

    // Port 0 : position
    i2c_write(0xFF);
    
    // Delay to handle the brightness
    _delay_ms(brightness[1] * 0.1);

    // Stop i2c
    i2c_stop();
}

void    display_one_digit(uint8_t number, uint8_t position, uint8_t dot){
    /*
    ** Displays one digit on a certain position
    */

    clear_all_digits();

    // Start
    i2c_start(EXPANDER_W, TW_MT_SLA_ACK);
    
    // Write to output port 0 and 1
    i2c_write(0x02);

    // Port 0 : position
    i2c_write((0xFF >> position) | (0xFF << (9 - position)));

    // Port 1 : digit
    i2c_write((dot)? numbers[number] | 0x80 : numbers[number]);
    
    // Delay to handle the brightness
    _delay_ms(brightness[0] * 0.1);

    // Stop i2c
    i2c_stop();
}

void    display_one_sign(uint8_t sign, uint8_t position, uint8_t dot){
    /*
    ** Displays one digit on a certain position
    */

    clear_all_digits();

    // Start
    i2c_start(EXPANDER_W, TW_MT_SLA_ACK);
    
    // Write to output port 0 and 1
    i2c_write(0x02);

    // Port 0 : position
    i2c_write((0xFF >> position) | (0xFF << (9 - position)));

    // Port 1 : digit
    i2c_write((dot) ? sign | 0x80 : sign);
    
    // Delay to handle the brightness
    _delay_ms(brightness[0] * 0.1);

    // Stop i2c
    i2c_stop();
}