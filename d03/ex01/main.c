#include <avr/io.h>
#ifndef F_CPU
# define F_CPU 16000000UL
#endif
#define UART_BAUDRATE 115200
#define __INTR_ATTRS used, externally_visible
#define ISR(vector, ...)            \
        void vector (void) __attribute__ ((signal,__INTR_ATTRS)) __VA_ARGS__; \
        void vector (void)

// Helper functions
uint32_t    my_log2(uint32_t n) {
    int result = 0;

    n = n / 2;
    while (n) {
        ++result;
        n = n / 2;
    }
    return result;
}

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
    // Enable transmitter (not receiver) + TX interrupts
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

// Interrupt Service Routine of timer 0 that triggers on compare match
ISR(TIMER1_COMPA_vect)
{
    /*
    ** This is triggered every time timer1 comp A match
    */

    // Transmit hello world!
    uart_printstr("Hello World!");
}

void    timer_1_conf(uint16_t prescale, uint8_t int_delay) {
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
    OCR1A = (F_CPU / (2 * 256 * (int_delay / 4)) - 1);
    // 4. Enable and configure timer 1 interrupt
    TIMSK1 |= (1 << OCIE1A);
    // 4. Clock prescale factor + launch the timer
    TCCR1B |= ((uint32_t)(my_log2(prescale) / 2) << CS10);
}

int main(void){

    // Initialize UART
    uart_init(UART_BAUDRATE, 0);

    // Initialize Timer 1 with interrupt every 2 seconds
    timer_1_conf(256, 2);

    // Initialize global interrupt
    SREG |= (1 << 7); // sei(); is also valid but I like the "raw" way

    while (1) 
        ;

    return (0);
}
