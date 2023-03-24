// AVR
#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>
#ifndef F_CPU
# define F_CPU 16000000UL
#endif
// ADC
#define RV1 0
#define LDR 1
#define NTC 2
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
volatile uint16_t       value = 0;
static const uint8_t    brightness[] = {30, 10}; // ratio on / off on the 7seg
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
static const uint8_t    numbers[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, \
                                     0x07, 0x7F, 0x6F};


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
** ADC Functions
** -----------------------------------------------------------------------------
*/
void    adc_init(void){
    /*
    ** Configure and initialize ADC (Analog to Digital Converter)
    ** 
    ** All configs are explicits here for learning purposes (even if I don't 
    ** need to write a 0 if it is the default value).
    */

    // Select Reference Voltage (AVcc here) to compare the analog signal with
    ADMUX |= (1 << REFS0);

    // Configure the data register on left (ADLAR 1) or right (ADLAR 0) adjust
    // ADMUX |= (1 << ADLAR); // Adjust right so we just read ADC for 10bits

    // Select proper pin (RV1)
    ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0));

    // Enable ADC
    ADCSRA |= (1 << ADEN);

    // Enable ADC Interrupts
    ADCSRA |= (1 << ADIE);

    // Configure prescaler (128 division factor here)
    ADCSRA |= (7 << ADPS0);

    // Disable digital input buffer on the POT pin (useless as we use ADC data
    // registers)
    DIDR0 |= (1 << ADC0D);
}

void    start_conversion(void){
    /*
    ** Explicitly start an ADC conversion for RV1
    */
 
    // Start conversion
    ADCSRA |= (1 << ADSC);
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
void    clear_all_positions(void){
    /*
    ** Clear 7seg
    */

    // Start
    i2c_start(EXPANDER_W, TW_MT_SLA_ACK);
    if (i2c_error) { return; }
    
    // Write to output port 0 and 1
    i2c_write(0x02);
    if (i2c_error) { return; }

    // Port 0 : position
    i2c_write(0xFF);
    if (i2c_error) { return; }
    // Delay to handle the brightness
    _delay_ms(brightness[1] * 0.1);

    // Stop i2c
    i2c_stop();
}

void    display_one_digit_7seg(uint8_t number, uint8_t position){
    /*
    ** Displays one digit on a certain position
    */

    clear_all_positions();

    // Start
    i2c_start(EXPANDER_W, TW_MT_SLA_ACK);
    if (i2c_error) { return; }
    
    // Write to output port 0 and 1
    i2c_write(0x02);
    if (i2c_error) { return; }

    // Port 0 : position
    i2c_write((0xFF >> position) | (0xFF << (9 - position)));
    if (i2c_error) { return; }

    // Port 1 : digit
    i2c_write(numbers[number]);
    if (i2c_error) { return; }
    // Delay to handle the brightness
    _delay_ms(brightness[0] * 0.1);

    // Stop i2c
    i2c_stop();
}

/*
** -----------------------------------------------------------------------------
** Timers
** -----------------------------------------------------------------------------
*/
void    timer_0_conf(uint16_t prescale) {
    /*
    ** This function configures the timer 0.
    */

    // 1. Waveform Generation Mode: CTC
    TCCR0A |= (1 << WGM01);
    // 3. Value to compare the timer with
    OCR0A = 0xFF;
    // 3. Interrupts when compare match
    TIMSK0 |= (1 << OCIE0A);
    // 4. Clock prescale factor
    TCCR0B |= ((uint32_t)(my_log2(prescale) / 2) << CS00);
}

/*
** -----------------------------------------------------------------------------
** Interrupts
** -----------------------------------------------------------------------------
*/
ISR(TIMER0_COMPA_vect){
    /*
    ** Timer to update digit and display it on 7 seg
    */
    display_one_digit_7seg(value % 10, 1);
    display_one_digit_7seg((value / 10) % 10, 2);
    display_one_digit_7seg((value / 100) % 10, 3);
    display_one_digit_7seg((value / 1000) % 10, 4);
}

ISR(ADC_vect){
    /*
    ** ADC conversion complete
    */
    uart_printstr("OK\n\r");
    // Read the value of ADC register and update global variable with it
    value = ADC;
    // Clear the ADIF flag with logical 1 to disable pending interrupts
    ADCSRA |= (1 << ADIF);
    // Start a new conversion
    start_conversion();
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

    // Initialize ADC and start the first conversion
    adc_init();
    start_conversion();

    // Start a timer to refresh the 7sec
    timer_0_conf(1024);

    // Loop
    while (1) {
    }

    return (0);
}
