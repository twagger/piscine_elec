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
// RGB
#define DEL_RED PD5
#define DEL_GREEN PD6
#define DEL_BLUE PD3
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
volatile const uint32_t rgb[7] = {RED, GREEN, BLUE, YELLOW, CYAN, MAGENTA, \
                                  WHITE};
volatile uint8_t        current_color = 0;

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
** RGD LED
** -----------------------------------------------------------------------------
*/

void    display_rgb(uint32_t rgb){
    /*
    ** Convert a rgb color on 24 bits (FFFFFF) to a led response
    */

    // Set all color off
    PORTD &= ~((1 << DEL_RED) | (1 << DEL_GREEN) | (1 << DEL_BLUE));

    // RED
    if ((rgb & RED) == RED) { PORTD |= (1 << DEL_RED); }

    // GREEN
    if ((rgb & GREEN) == GREEN) { PORTD |= (1 << DEL_GREEN); }

    // BLUE
    if ((rgb & BLUE) == BLUE) { PORTD |= (1 << DEL_BLUE); }

}

void    rotate_rgb(void){
    /*
    ** Rotate the color following the color array
    */

    const uint8_t   nb_rgb = sizeof(rgb) / sizeof(rgb[0]);

    // Display current color
    display_rgb(rgb[current_color]);

    // Rotate color for next loop
    current_color = (current_color + 1 >= nb_rgb) ? 0 : current_color + 1;

}

void    init_rgb(void){
    /*
    ** Set the led as output and turn them off
    */

    const uint8_t   nb_rgb = sizeof(rgb) / sizeof(rgb[0]);

    for (uint8_t i = 0; i < nb_rgb; i++) {
        DDRD |= (1 << rgb[i]); // Output
        PORTD &= ~(1 << rgb[i]); // Off by default
    }
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
    rotate_rgb();
}

/*
** -----------------------------------------------------------------------------
** Timers
** -----------------------------------------------------------------------------
*/

void    timer_1_conf(uint16_t prescale) {
    /*
    ** This function configures the timer 1 to act on PB1 in a certain way 
    ** depending on a duty cycle (managed by OCR1A register).
    ** For the effect to be visible, this timer should update PB1 every
    */

    // 1. Action on OC1A (PB1) on compare match. Here : nothing
    TCCR1A &= ~((1 << COM1A0) | (1 << COM1A1));
    // 2. Waveform Generation Mode: CTC
    TCCR1B |= (1 << WGM12);
    // 3. Value to compare the timer with
    OCR1A = (F_CPU / (2 * 256) - 1);
    // 4. Enable and configure timer 1 interrupt
    TIMSK1 |= (1 << OCIE1A);
    // 4. Clock prescale factor + launch the timer
    TCCR1B |= ((uint32_t)(my_log2(prescale) / 2) << CS10);
}

/*
** -----------------------------------------------------------------------------
** MAIN program
** -----------------------------------------------------------------------------
*/
int main(void){

    // Initialize UART
    uart_init(UART_BAUDRATE, 0);

    // Init RGB leds
    init_rgb();

    // Enable global interrupts
    SREG |= (1 << 7);

    // Init Timer 1 to produce 1 interrupt per second
    timer_1_conf(256);
    
    while (1) {

    }

    return (0);
}
