#include "i2c.h"

/*
** -----------------------------------------------------------------------------
** I2C Expander Functions
** -----------------------------------------------------------------------------
*/
void    init_i2e_expander(uint8_t port0, uint8_t port1){
    /*
    ** Initializes i2c expander
    */

    // Start transmission with the E2C IO expander
    i2c_start(EXPANDER_W, TW_MT_SLA_ACK);
    
    // Configure SW3 as input and DELS as output
    i2c_write(6); // configuration port 0

    // Configuration data
    i2c_write(port0); // All output but the sw3

    i2c_write(port1); // All output

    // Stop i2c
    i2c_stop();
}
