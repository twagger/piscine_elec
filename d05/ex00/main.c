// AVR
#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#ifndef F_CPU
# define F_CPU 16000000UL
#endif
// Switch
#define DEBOUNCING 250 // debouncing delay (in ms)
// EEPROM
#define ADDRESS 0
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
volatile uint8_t        counter;
volatile const uint8_t  leds[] = {PB0, PB1, PB2, PB4};

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

void    counter_setup(uint8_t address){
    // Read counter current state
    counter = eeprom_read_byte((uint8_t *)0);
    // Display binary value on leds
    display_bin(counter);
    // This enables to reset the counter with two consecutive resets
    eeprom_write_byte(0, 0);
}

void    ft_byte_printer(unsigned char c) {
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
** Interrupts
** -----------------------------------------------------------------------------
*/
ISR(INT0_vect) {
    /*
    ** ISR of SW1
    */

    // Here I don't read counter because it has been loaded by counter_setup()
    ft_byte_printer(counter);
    counter++;
    eeprom_write_byte((uint8_t *)0, counter);
    display_bin(counter);
    _delay_ms(DEBOUNCING);
    EIFR = (1 << INTF0); // clear external interrupt flag 
} 

/*
** -----------------------------------------------------------------------------
** EEPROM Functions
** -----------------------------------------------------------------------------
*/
// If I don't use eeprom.h, below is the code for read / write a byte with 
// eeprom
//
// void EEPROM_write(uint8_t address, unsigned char data){
//     /*
//     ** Writes the data in the address
//     */

//     // Wait for completion of previous write
//     while(EECR & (1 << EEPE))
//         ;
//     // Set up address and Data Registers
//     EEAR = address;
//     EEDR = data;

//     // Write logical one to EEMPE : Master write enable
//     EECR |= (1 << EEMPE);
//     // Start eeprom write by setting EEPE
//     EECR |= (1 << EEPE);
// }

// unsigned char   EEPROM_read(uint8_t address){
//     /*
//     ** Reads the address and returns the data
//     */

//     // Wait for completion of previous write */
//     while(EECR & (1<<EEPE))
//         ;
//     // Set up address register
//     EEAR = address;
//     // Start eeprom read by writing EERE
//     EECR |= (1 << EERE);
//     // Return data from Data Register
//     return EEDR;
// }

/*
** -----------------------------------------------------------------------------
** MAIN program
** -----------------------------------------------------------------------------
*/
int main(void){

    // Initialize UART
    uart_init(UART_BAUDRATE, 0);

    // Init SW1 interrupt + global interrupts
    int0_init();
    SREG |= (1 << 7); // sei(); is also valid but I like the "raw" way

    // Setup counter
    counter_setup(ADDRESS);

    while (1) {

    }

    return (0);
}
