#include "i2c.h"

/*
** -----------------------------------------------------------------------------
** I2C SW3 Functions
** -----------------------------------------------------------------------------
*/
uint8_t check_sw3(void){
    /*
    ** Reads from E2C IO expander to check if SW3 is pressed
    */

    uint8_t reg_value;

    // Start in Write mode to specify what we want to read
    i2c_start(EXPANDER_W, TW_MT_SLA_ACK);
    if (i2c_error) { return (2); }

    // Send register address that we want to read
    i2c_write(0); // Input port register
    if (i2c_error) { return (2); }

    // Start in read mode to read the data from register
    i2c_repeat_start(EXPANDER_R, TW_MR_SLA_ACK);
    if (i2c_error) { return (2); }

    // Send NACK to stop after one byte sent from I2C expander
    send_nack();
    if (i2c_error) { return (2); }

    // Read the value of register
    reg_value = i2c_read();

    // Stop i2c
    i2c_stop();

    return (!(reg_value & 1));
}
