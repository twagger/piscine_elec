// AVR
#include <avr/io.h>
#include <util/twi.h>
#ifndef F_CPU
# define F_CPU 16000000UL
#endif
#define SENSOR_ADDR  0b00111000

// Helper functions
void    wait_for_transmission(void){
    while (!(TWCR & (1 << TWINT)))
        ;
}

int check_for_status(int expected){
    if ((TWSR & 0xF8) != expected){
        // push error to screen
        // stops the program
        return (1);
    }
    else {
        // push status to screen
    }
    return (0);
}

// I2C Functions
void    i2c_init(void){
    /*
    ** Initializes I2C on the MCU.
    **
    ** - Communication frequency : 100kHz
    */

    // Enable ACK bit
    // Maybe to do 
    // TWCR |= (1 << TWEA);

    // Configure prescale and bit rate for 100kHz
    TWSR &= ~((1 << TWPS0) | (1 << TWPS1));
    TWBR = 72;
}

void i2c_start(void){
    /*
    ** Starts an I2C transmission between the MCU and the sensor. Prepare it in 
    ** write mode. The bus is then considered busy for others.
    */

    uint8_t sla_w;

    // Send start condition and clear the TWINT flag (to check then when the 
    // task is done)
    TWCR |= (1 << TWSTA) | (1 << TWINT) | (1 << TWEN);
    wait_for_transmission();
    if (check_for_status(TW_START) == 1) {
        // error
    }

    // Load the slave address + write mode in data register
    sla_w = (SENSOR_ADDR << 1);
    TWDR = sla_w;

    // Transmit the address
    TWCR = (1 << TWINT) | (1 << TWEN);
    wait_for_transmission();
    if (check_for_status(TW_MT_SLA_ACK) == 1) {
        // error
    }

}


void i2c_stop(void){
    /*
    ** Stops the I2C communication between the MCU and the sensor. The bus is no
    ** longer busy.
    */

    // Send stop condition and clear the TWINT flag (to check then when the 
    // task is done)
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}


// Main
int main(void){
    
    // Init I2C
    i2c_init();

    // Start transmission
    i2c_start();

    // Stop transmission
    i2c_stop();

    while (1) {
       
    }

    return (0);
}
