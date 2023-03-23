
// AVR
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#ifndef F_CPU
# define F_CPU 16000000UL
#endif
// UART
#define UART_BAUDRATE 115200
#define __INTR_ATTRS used, externally_visible
#define ISR(vector, ...)            \
        void vector (void) __attribute__ ((signal,__INTR_ATTRS)) __VA_ARGS__; \
        void vector (void)
// SPI – Serial Peripheral Interface
#define MOSI PB3
#define MISO PB4
#define SCK PB5
#define SS PB2
// Colors
#define RED 0xFF0000
#define GREEN 0x00FF00
#define BLUE 0x0000FF
#define YELLOW 0xFFFF00
#define CYAN 0x00FFFF
#define MAGENTA 0xFF00FF
#define WHITE 0xFFFFFF


/*
** -----------------------------------------------------------------------------
** Globals
** -----------------------------------------------------------------------------
*/
volatile const uint32_t color[] = {RED, GREEN, BLUE, YELLOW, CYAN, MAGENTA, \
                                   WHITE};
const uint8_t           nb_colors = sizeof(color) / sizeof(color[0]);
volatile uint8_t        current_color = 0;


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
** SPI
** -----------------------------------------------------------------------------
*/
void    spi_wait(void) {
    /*
    ** Waits for transmission flag to be set
    */
   
    // Wait until the SPIF is cleared
    while (!(SPSR & (1 << SPIF)));
    // Read what has been send by the slave to clear SPIF
    uint8_t trash = SPDR;
}

void    spi_master_init(void){
    /*
    ** Initialize the SPI connection as Master
    */

    // Set MOSI and SCK output, SS and MISO inputs
    DDRB = (1 << MOSI) | (1 << SCK) | (1 << SS);    // Outputs
    DDRB &= ~(1 << MISO);                           // Inputs

    // Data order : MSB or LSB
    SPCR |= (1 << DORD); // LSB first

    // Clock polarity
    SPCR |= (1 << CPOL); // High when idle

    // Clock phase : is data sampled on the leading or trailing edge of SCK
    SPCR |= (1 << CPHA); // trailing edge

    // Clock speed 
    SPCR |= (1 << SPR0); // (Fosc / 16)

    // Master mode
    SPCR |= (1 << MSTR);

    // Enable SPI
    SPCR |= (1 << SPE);
}

void    spi_write(uint8_t byte){
    /*
    ** Write one byte to spi slave
    */

    // Set SS to 0
    PORTB &= ~(1 << SS);

    // Write a byte to Data Register
    SPDR = byte;
    spi_wait();

    // Set SS to 1 to resynch the slave after the byte has been sent
    PORTB |= (1 << SS);
}

/*
** -----------------------------------------------------------------------------
** Program helpers
** -----------------------------------------------------------------------------
*/
void    spi_apa102_start_frame(void){
    // Start frame (0*32)
    spi_write((uint8_t)0);
    spi_write((uint8_t)0);
    spi_write((uint8_t)0);
    spi_write((uint8_t)0);
}

void    spi_apa102_end_frame(void){
    // Start frame (1*32)
    spi_write((uint8_t)1);
    spi_write((uint8_t)1);
    spi_write((uint8_t)1);
    spi_write((uint8_t)1);
}

void    spi_d6(uint32_t color, uint8_t brightness){
    /*
    ** Turn on D6 RGB led in red
    */

    spi_apa102_start_frame();

    // display color with d6
    spi_write((uint8_t)(7 << 5) | (brightness & 0x1F));
    spi_write((uint8_t)(color & 0xFF));
    spi_write((uint8_t)(color >> 8) & 0xFF);
    spi_write((uint8_t)(color >> 16) & 0xFF);

    spi_apa102_end_frame();
}

/*
** -----------------------------------------------------------------------------
** Timers
** -----------------------------------------------------------------------------
*/
void    timer_1_conf(uint16_t prescale, uint8_t frequency) {
    /*
    ** Interrupt every 20ms
    */

    // 1. Action on OC1A (PB1) on compare match. Here : nothing
    TCCR1A &= ~((1 << COM1A0) | (1 << COM1A1));
    // 2. Waveform Generation Mode: CTC
    TCCR1B |= (1 << WGM12);
    // 3. Value to compare the timer with
    OCR1A = (F_CPU / (2 * 256 * frequency) - 1);
    // 4. Enable and configure timer 1 interrupt
    TIMSK1 |= (1 << OCIE1A);
    // 4. Clock prescale factor + launch the timer
    TCCR1B |= ((uint32_t)(my_log2(prescale) / 2) << CS10);
}

/*
** -----------------------------------------------------------------------------
** Interrupts
** -----------------------------------------------------------------------------
*/

ISR(TIMER1_COMPA_vect)
{
    /*
    ** This is triggered every time timer1 comp A match
    */

    // Change color of the led
    spi_d6(color[current_color], 0x1F);
    current_color = (current_color + 1 > nb_colors) ? 0 : current_color + 1;
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

    // Initialize SPI as Master
    spi_master_init();

    // Init timer 1 to produce 1 interrupt every second (1Hz frequency)
    timer_1_conf(256, 1);

    // Loop
    while (1) {

    }

    return (0);
}
