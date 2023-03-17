// AVR
#include <avr/io.h>
#include <util/twi.h>
#ifndef F_CPU
# define F_CPU 16000000UL
#endif
// I2C
#define SENSOR_ADDR  0b00111000
// UART
#define UART_BAUDRATE 115200
#define __INTR_ATTRS used, externally_visible
#define ISR(vector, ...)            \
        void vector (void) __attribute__ ((signal,__INTR_ATTRS)) __VA_ARGS__; \
        void vector (void)

// Globals
static uint8_t  i2c_error = 0;

// UART functions
void    uart_init(uint32_t baud, uint8_t double_speed) {
    /*
    ** This function initializes UART. The standard steps for init are :
    ** 
    ** - Setting the baud rate
    ** - Setting frame format
    ** - Enabling the tranmitter or receiver (depending on the usage)
    ** - Clear global interrupt flag (for interrupt driven USART operation)
    */

    uint32_t    ubrr;
    uint8_t     speed;

    speed = (double_speed) ? 8 : 16;

    // Calculation of the UBRR0 : USART Baud Rate Registers
    // - UBRR0H : contains the four most significant bits of the baud rate
    // - UBRR0L : contains the eight least significant bits of the baud rate
    ubrr = ((F_CPU / (speed / 2) / baud) - 1) / 2;
    UBRR0H = (ubrr & 0x0F00) >> 8; // Picking 4 most significant bits
    UBRR0L = (ubrr & 0x00FF); // Picking 8 least significant bits
    // Set frame format: 8 bits data, No parity bits, 1 stop bit (8N1)
    UCSR0C = (3 << UCSZ00);
    UCSR0C &= ~((1 << UPM01) | (1 << UPM00));
    UCSR0C &= ~(1 << USBS0);
    // Enable transmitter
    UCSR0B = (1 << TXEN0);
}

void    uart_tx(char c) {
    /*
    ** This function transmit a char through UART
    */

    // Wait for any previous data to be transfered
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    // Put data into buffer, sends the data
    UDR0 = c;
}

void    uart_printstr(const char *str) {
    while (*str){
        uart_tx(*str);
        ++str;
    }
}

// I2C Functions
void    wait_for_transmission(void){
    /*
    ** Waits for transmission to be ok by checking TWINT bit in Control registry
    */

    while (!(TWCR & (1 << TWINT)))
        ;
}

int check_for_status(int expected){
    /*
    ** Checks for I2C status in TWSR registry
    */

    if ((TWSR & 0xF8) != expected){
        // push error to screen
        uart_printstr("Error");
        i2c_error = 1;
        return (1);
    }
    else {
        if (expected == TW_START) {
            uart_printstr("A START condition has been transmitted\n\r");
        }
        else if (expected == TW_MT_SLA_ACK) {
            uart_printstr("SLA+W has been transmitted; ");
            uart_printstr("ACK has been received\n\r");
        }
    }
    return (0);
}

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
        return;
    }

    // Load the slave address + write mode in data register
    sla_w = (SENSOR_ADDR << 1);
    TWDR = sla_w;

    // Transmit the address
    TWCR = (1 << TWINT) | (1 << TWEN);
    wait_for_transmission();
    if (check_for_status(TW_MT_SLA_ACK) == 1) {
        return;
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
    
    // Initialize UART to transfer sensor and I2C infos
    uart_init(UART_BAUDRATE, 0);

    // Init I2C
    i2c_init();

    // Start transmission
    i2c_start();

    // Check for errors
    if (i2c_error) {
        return (1);
    }

    // Stop transmission
    i2c_stop();

    // Check for errors
    if (i2c_error) {
        return (1);
    }

    while (1) {
       
    }

    return (0);
}
