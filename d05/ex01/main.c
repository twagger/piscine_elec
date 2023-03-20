// AVR
#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/twi.h>
#ifndef F_CPU
# define F_CPU 16000000UL
#endif
// Switch
#define DEBOUNCING 250 // debouncing delay (in ms)
// I2C
#define SWITCH_W 0b01000000
#define SWITCH_R 0b01000001
// EEPROM
#define ADDRESS 0
#define NB_COUNTERS 4
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
volatile uint8_t        counter[NB_COUNTERS];
volatile uint8_t        selected = 0;
volatile const uint8_t  leds[] = {PB0, PB1, PB2, PB4};
static uint8_t          i2c_error = 0;
static const uint8_t    debug = 0;
static const char       *status_mess[] = {\
                             "A START condition has been transmitted", \
                             "SLA+W has been transmitted; ACK received", \
                             "SLA+R has been transmitted; ACK received", \
                             "Data byte has been transmitted; ACK received", \
                             "A repeated START condition has been transmitted"};
static const uint8_t    status_codes[] = {TW_START, TW_MT_SLA_ACK,
                                          TW_MR_SLA_ACK, TW_MT_DATA_ACK,
                                          TW_REP_START};

// Enums
enum    e2i_options{
    OPT_DATA = 0,
    OPT_START,
    OPT_STOP,
};

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
** Helper Functions
** -----------------------------------------------------------------------------
*/
void    display_bin(uint8_t number){
    /*
    ** Display the 4 LSB of a number using 4 leds of the board
    */

    const uint8_t   nb_leds = sizeof(leds) / sizeof(leds[0]);

    for (uint8_t i = 0; i < nb_leds; i++) {
        if (number & 1 << i){
            PORTB |= (1 << leds[i]);
        }
        else { PORTB &= ~(1 << leds[i]); }
    }
}

void    int0_init(void){
    /*
    ** Enable and conf interruptions for SW1
    */

    // Parameterization of the interrupt
    // 1. Sense control : low level
    // As bouncing will generates more edges than level, it is safer to chose
    // the low level instead of the falling or rising edge events.
    EICRA &= ~((1 << ISC00) | (1 << ISC01));
    // 2. Enable INT0 interrupt
    EIMSK |= (1 << INT0);
    // 3. Enable global interrupt by setting global interrupt enable bit to 1
    SREG |= (1 << 7); // sei(); is also valid
}

void    pcint20_init() {
    /*
    ** Parameterization of the PCINT20 interrupt
    ** Carefull : ANY change will trigger the associated ISR (no sense control)
    */

    // Enable PCINT23:16 range interrupts
    PCICR |= (1 << PCIE2);
    // Enable PCINT20 interrupts only
    PCMSK2 = (1 << PCINT20);
}

void    counters_setup(void){
    // Read counters current state
    for (uint8_t i = 0; i < NB_COUNTERS; i++){
        counter[i] = eeprom_read_byte((uint8_t *)(0 + i));
    }
    // Display binary value on leds for currently selected counter
    display_bin(counter[selected]);
    // This enables to reset the counter with two consecutive resets
    for (uint8_t i = 0; i < NB_COUNTERS; i++){
        eeprom_write_byte((uint8_t *)(0 + i), 0);
    }
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
        uart_printstr("Error");
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
        TWCR |= (1 << TWSTA) | (1 << TWINT) | (1 << TWEN);
        wait_for_transmission();
        if (check_for_status(expected) == 1) {
            return;
        }
    }
    else if (option == OPT_STOP) {
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
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

    // Enable ACK bit
    // Maybe to do 
    // TWCR |= (1 << TWEA);

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
    if (i2c_error) { return; }
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
** SW3 Functions
** -----------------------------------------------------------------------------
*/
// void    init_sw3(void){
//     /*
//     ** Initializes i2c with SW3
//     */

//     // Start transmission with the E2C IO expander
//     i2c_start(SWITCH_W, TW_MT_SLA_ACK);
//     if (i2c_error) { return; }

//     // Configure SW3 as input
//     i2c_write(0x06); // configuration port 0
//     if (i2c_error) { return; }

//     // Configuration data
//     i2c_write(0xFF); // pin 0_0 (SW3) as input
//     if (i2c_error) { return; }
    
//     // Stop i2c
//     i2c_stop();
// }

// uint8_t check_sw3(void){
//     /*
//     ** Reads from E2C IO expander to check if SW3 is pressed
//     */

//     uint8_t result = 0;
//     uint8_t reg_value;

//     // Start in Write mode to specify what we want to read
//     i2c_start(SWITCH_W, TW_MT_SLA_ACK);
//     if (i2c_error) { return (2); }

//     // Send register address that we want to read
//     i2c_write(0x00); // Input port register
//     if (i2c_error) { return (2); }

//     // Start in read mode to read the data from register
//     i2c_repeat_start(SWITCH_R, TW_MR_SLA_ACK);
//     if (i2c_error) { return (2); }

//     // Read the value of register
//     reg_value = i2c_read();
    
//     uart_byte_printer(reg_value);
//     _delay_ms(2000);

//     // Check if SW3 is pressed
//     if ((reg_value & (1 << 0)) == 0){ // SW3 is pressed
//         result = 1;
//     }

//     // Send NACK
//     send_nack();
//     if (i2c_error) { return (2); }
//     // Stop i2c
//     i2c_stop();
//     return (result);
// }

/*
** -----------------------------------------------------------------------------
** Interrupts
** -----------------------------------------------------------------------------
*/
ISR(INT0_vect) {
    /*
    ** ISR of SW1
    */

    // Here I don't read counter because it has been loaded by counter_setup()
    counter[selected]++;
    uart_byte_printer(counter[selected]);
    eeprom_write_byte((uint8_t *)(0 + selected), counter[selected]);
    display_bin(counter[selected]);
    _delay_ms(DEBOUNCING);
    EIFR = (1 << INTF0); // clear external interrupt flag 
}

ISR(PCINT2_vect){
    /*
    ** This function will change the counter a counter when it is triggered
    */

    if ((PIND & (1 << PD4)) == 0) { // I use PINX to be clean (could use global)
        selected = (selected + 1 >= NB_COUNTERS) ? 0 : selected + 1;
        display_bin(counter[selected]);
        _delay_ms(DEBOUNCING);
        EIFR = (1 << INTF0); // clear external interrupt flag 
    }
}

/*
** -----------------------------------------------------------------------------
** MAIN program
** -----------------------------------------------------------------------------
*/
int main(void){

    // uint8_t sw3_status;

    // Initialize UART
    uart_init(UART_BAUDRATE, 0);

    // Init SW1 interrupt + SW2 interrupt + enable global interrupts
    int0_init();
    pcint20_init();
    SREG |= (1 << 7);

    // Init SW3 I2C
    // i2c_init();
    // init_sw3();
    // if (i2c_error) { return (1); }

    // Setup counter
    counters_setup();

    display_bin(counter[selected]);

    while (1) {
    }

    return (0);
}
