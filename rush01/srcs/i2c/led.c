#include "i2c.h"

/*
** -----------------------------------------------------------------------------
** I2C Leds Functions
** -----------------------------------------------------------------------------
*/
void    toogle_i2c_led(uint8_t led){
    /*
    ** Toogle I2C Led
    */
    uint8_t state;

    // Start in Write mode to specify what we want to read
    i2c_start(EXPANDER_W, TW_MT_SLA_ACK);

    // Send the output register that we want to read
    i2c_write(0x02);

    // Repeat start to switch on read mode
    i2c_repeat_start(EXPANDER_R, TW_MR_SLA_ACK);

    // Send NACK to stop after one byte sent from I2C expander
    send_nack();

    // Read the value of register and toogle the led value
    state = i2c_read();
    state ^= (1 << led);

    // Repeat start to switch on write mode
    i2c_repeat_start(EXPANDER_W, TW_MT_SLA_ACK);

    // Write to output register
    i2c_write(0x02);

    // Write output 0
    i2c_write(state);

    // Stop i2c
    i2c_stop();
}

void    turn_on_i2c_led(uint8_t led){
    /*
    ** Turn on I2C Led
    */
    uint8_t state;

    // Start in Write mode to specify what we want to read
    i2c_start(EXPANDER_W, TW_MT_SLA_ACK);

    // Send the output register that we want to read
    i2c_write(0x02);

    // Repeat start to switch on read mode
    i2c_repeat_start(EXPANDER_R, TW_MR_SLA_ACK);

    // Send NACK to stop after one byte sent from I2C expander
    send_nack();

    // Read the value of register and toogle the led value
    state = i2c_read();
    state |= (1 << led);

    // Repeat start to switch on write mode
    i2c_repeat_start(EXPANDER_W, TW_MT_SLA_ACK);

    // Write to output register
    i2c_write(0x02);

    // Write output 0
    i2c_write(state);

    // Stop i2c
    i2c_stop();
}

void    turn_off_i2c_led(uint8_t led){
    /*
    ** Turn on I2C Led
    */
    uint8_t state;

    // Start in Write mode to specify what we want to read
    i2c_start(EXPANDER_W, TW_MT_SLA_ACK);

    // Send the output register that we want to read
    i2c_write(0x02);

    // Repeat start to switch on read mode
    i2c_repeat_start(EXPANDER_R, TW_MR_SLA_ACK);

    // Send NACK to stop after one byte sent from I2C expander
    send_nack();

    // Read the value of register and toogle the led value
    state = i2c_read();
    state &= ~(1 << led);

    // Repeat start to switch on write mode
    i2c_repeat_start(EXPANDER_W, TW_MT_SLA_ACK);

    // Write to output register
    i2c_write(0x02);

    // Write output 0
    i2c_write(state);

    // Stop i2c
    i2c_stop();
}
