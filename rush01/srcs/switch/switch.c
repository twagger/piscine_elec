#include "switch.h"
#include "uart.h" // for debug
/*
** -----------------------------------------------------------------------------
** SWITCH Functions
** -----------------------------------------------------------------------------
*/

void    switch1_init(uint8_t sense_control){ //INT0
    /*
    ** Enable and conf interruptions for SW1 (INT0)
    */
 
    // Parameterization of the interrupt
    // 1. Sense control : low level
    // As bouncing will generates more edges than level, it is safer to chose
    // the low level instead of the falling or rising edge events.
    EICRA &= ~((1 << ISC00) | (1 << ISC01));
    // EICRA |= (sense_control << ISC00);
    // 2. Enable INT0 interrupt
    EIMSK |= (1 << INT0);
}

void    switch2_init() { // PCINT20
    /*
    ** Parameterization of the PCINT20 interrupt
    ** Carefull : ANY change will trigger the associated ISR (no sense control)
    */

    // Enable PCINT23:16 range interrupts
    PCICR |= (1 << PCIE2);
    // Enable PCINT20 interrupts only
    PCMSK2 = (1 << PCINT20);
}

uint8_t switch3_check(void){
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
