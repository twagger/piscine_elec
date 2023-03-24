// AVR
#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>
#ifndef F_CPU
# define F_CPU 16000000UL
#endif
// I2C
#define EXPANDER_W 0b01000000
#define EXPANDER_R 0b01000001
#define OPT_DATA 0
#define OPT_START 1
#define OPT_STOP 2
// Switch
#define DEBOUNCING 250 // debouncing delay (in ms)
// UART
#define UART_BAUDRATE 115200
#define __INTR_ATTRS used, externally_visible
#define ISR(vector, ...)            \
        void vector (void) __attribute__ ((signal,__INTR_ATTRS)) __VA_ARGS__; \
        void vector (void)

/*
** -----------------------------------------------------------------------------
** Globals
** -----------------------------------------------------------------------------
*/
volatile const uint8_t  i2c_leds[] = {1, 2, 3};
static uint8_t          i2c_error = 0;
static const uint8_t    debug = 0;
static const char       *status_mess[] = {\
                      "A START condition has been transmitted", \
                      "SLA+W has been transmitted; ACK received", \
                      "SLA+R has been transmitted; ACK received", \
                      "Data byte has been transmitted; ACK received", \
                      "A repeated START condition has been transmitted", \
                      "Data byte has been received; NOT ACK has been returned"};
static const uint8_t    status_codes[] = {TW_START, TW_MT_SLA_ACK,
                                          TW_MR_SLA_ACK, TW_MT_DATA_ACK,
                                          TW_REP_START, TW_MR_DATA_NACK};


/*
** -----------------------------------------------------------------------------
** Helper Functions
** -----------------------------------------------------------------------------
*/
uint32_t    my_log2(uint32_t n) {
    int result = 0;

    n = n / 2;
    while (n) {
        ++result;
        n = n / 2;
    }
    return result;
}

/*
** -----------------------------------------------------------------------------
** UART Functions
** -----------------------------------------------------------------------------
*/
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
    // Enable transmitter and receiver + TX interrupts on reception complete
    UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
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

unsigned char   uart_rx(void) {
    /*
    ** This function transmit a char through UART
    */

    // Wait for the data to be received
    while (!(UCSR0A & (1 << RXC0)))
        ;
    // Return data from buffer
    return UDR0;
}

void    uart_printstr(const char *str) {
    while (*str){
        uart_tx(*str);
        ++str;
    }
}

void    uart_byte_printer(unsigned char c) {
    /*
    ** for debbugging
    */

    char bit;

    for (uint8_t i = 0; i < 8; i++) {
        bit = (c << i & 0x80) ? 49 : 48;
        uart_tx(bit);
    }
    uart_tx('\n');
    uart_tx('\r');
}

/*
** -----------------------------------------------------------------------------
** I2C Functions
** -----------------------------------------------------------------------------
*/
void    wait_for_transmission(void) {
    /*
    ** Waits for transmission to be ok by checking TWINT bit in Control registry
    */

    while (!(TWCR & (1 << TWINT)))
        ;
}

int check_for_status(uint8_t expected){
    /*
    ** Checks for I2C status in TWSR registry
    */

    if ((TWSR & 0xF8) != expected){
        // push error to screen
        uart_printstr("Error\n\r");
        uart_byte_printer((TWSR & 0xF8));
        uart_byte_printer(expected);
        i2c_error = 1;
        return (1);
    }
    else if (debug == 1) {
        for (uint8_t i = 0; i < sizeof(status_codes) / sizeof(status_codes[0]);\
             i++) {
            if (expected == status_codes[i]) {
                uart_printstr((const char *)status_mess[i]);
                uart_printstr("\n\r");
            }
        }
    }
    return (0);
}

void    send_to_i2c(uint8_t option, unsigned char data, uint8_t expected){
    /*
    ** Send to i2c bus
    */

    // START / STOP / RESTART
    if (option == OPT_START) {
        TWCR = (1 << TWINT) | (1 << TWSTA)| (1 << TWEN);
        wait_for_transmission();
        if (check_for_status(expected) == 1) {
            return;
        }
    }
    else if (option == OPT_STOP) {
        TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
        _delay_ms(0.01); // to be sure the bus is idle for the next start
    }
    else if (option == OPT_DATA) {
        // DATA
        // Load data register with data
        TWDR = data;
        // Transmit
        TWCR = (1 << TWINT) | (1 << TWEN);
        wait_for_transmission();
        if (check_for_status(expected) == 1) {
            return;
        }
    }
}

void    i2c_init(void){
    /*
    ** Initializes I2C on the MCU.
    **
    ** - Communication frequency : 100kHz
    */

    // Configure prescale and bit rate for 100kHz
    TWSR &= ~((1 << TWPS0) | (1 << TWPS1));
    TWBR = 72;
}

void    i2c_start(uint8_t data, uint8_t expected){
    /*
    ** Starts an I2C transmission between the MCU and the sensor. Prepare it in 
    ** write mode. The bus is then considered busy for others.
    */

    // Send start condition
    send_to_i2c(OPT_START, 0, TW_START);
    if (i2c_error) { return; }
        
    // Send slave address
    send_to_i2c(OPT_DATA, data, expected);
}

void    i2c_repeat_start(uint8_t data, uint8_t expected){
    /*
    ** Starts an I2C transmission between the MCU and the sensor. Prepare it in 
    ** write mode. The bus is then considered busy for others.
    */

    // Send start condition
    send_to_i2c(OPT_START, 0, TW_REP_START);
    if (i2c_error) { return; }
        
    // Send slave address
    send_to_i2c(OPT_DATA, data, expected);
}

void    i2c_write(unsigned char data){
    /*
    ** Write a byte to I2C bus
    */

    send_to_i2c(OPT_DATA, data, TW_MT_DATA_ACK);
}

uint8_t i2c_read(void){
    /*
    ** Display the content of TWDR after the sensor measurement
    */

    wait_for_transmission();
    return TWDR;
}

void    i2c_stop(void){
    /*
    ** Stops the I2C communication between the MCU and the sensor. The bus is no
    ** longer busy.
    */

    send_to_i2c(OPT_STOP, 0, 0);
    if (debug) {
        uart_printstr("A STOP condition has been transmitted\n\r\n\r");
    }
}

void    send_nack(void){
    TWCR = (1 << TWINT) | (1 << TWEN);
    wait_for_transmission();
    if (check_for_status(TW_MR_DATA_NACK) == 1) {
        return;
    }
}

/*
** -----------------------------------------------------------------------------
** I2C Expander Functions
** -----------------------------------------------------------------------------
*/
void    init_i2e_expander(void){
    /*
    ** Initializes i2c expander
    */

    // Start transmission with the E2C IO expander
    i2c_start(EXPANDER_W, TW_MT_SLA_ACK);
    if (i2c_error) { return; }
    
    // Configure SW3 as input and DELS as output
    i2c_write(6); // configuration port 0
    if (i2c_error) { return; }

    // Configuration data
    i2c_write(0b00000001); // All output but the sw3
    if (i2c_error) { return; }

    i2c_write(0b00000000); // All output
    if (i2c_error) { return; }

    // Stop i2c
    i2c_stop();
}

/*
** -----------------------------------------------------------------------------
** I2C 7 Segments
** -----------------------------------------------------------------------------
*/
void    display_one_digit_7seg(uint8_t number){
    /*
    ** Displays one digit (on the right)
    */

    // Overflow management
    number = (number > 9) ? 0 : number;

    // Start
    i2c_start(EXPANDER_W, TW_MT_SLA_ACK);
    if (i2c_error) { return; }
    
    // Write to output port 0 and 1
    i2c_write(0x02);
    if (i2c_error) { return; }

    // Port 0
    i2c_write(0b01111111);
    if (i2c_error) { return; }

    // Port 1
    i2c_write(0b11011011);
    if (i2c_error) { return; }

    // Stop i2c
    i2c_stop();
}

/*
** -----------------------------------------------------------------------------
** MAIN program
** -----------------------------------------------------------------------------
*/
int main(void){

    // Initialize UART
    uart_init(UART_BAUDRATE, 0);

    // Enable global interrupts
    SREG |= (1 << 7);

    // Init I2C expander
    i2c_init();
    init_i2e_expander();
    if (i2c_error) { return (1); }

    // Initial counter display
    display_one_digit_7seg(2);

    // Loop
    while (1) {
        
    }

    return (0);
}
