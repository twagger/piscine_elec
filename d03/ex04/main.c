#include <avr/io.h>
#ifndef F_CPU
# define F_CPU 16000000UL
#endif
#define UART_BAUDRATE 115200
#define BUFFER_SIZE 32
#define BACKSPACE 8
#define CR 13
#define LF 10
#define __INTR_ATTRS used, externally_visible
#define ISR(vector, ...) \
        void vector (void) __attribute__ ((signal,__INTR_ATTRS)) __VA_ARGS__; \
        void vector (void)

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
    // Enable global interrupts
    SREG |= (1 << 7);
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
    // Retuen data from buffer
    return UDR0;
}

void    uart_printstr(const char *str) {
    while (*str){
        uart_tx(*str);
        ++str;
    }
}

ISR(USART_RX_vect){
    unsigned char   c;
    static uint8_t  is_cr = 0;

    c = UDR0;
    if (c == BACKSPACE) {
        uart_tx((unsigned char)127)
    }
    else if (c == CR) {
        is_cr = 1;
    }
    else if (c == LF && is_cr == 1) {
        uart_tx('\n');
    }
    else {
        uart_tx(c);
    }
}

char    ask_username(char **in_username) {
    uart_printstr("Enter your login:\n");
    uart_printstr("\tusername:");
}

int main(void){

    char    username[BUFFER_SIZE] = {"twagner"};
    char    password[BUFFER_SIZE] = {"password"};
    char    in_username[BUFFER_SIZE];
    char    in_password[BUFFER_SIZE];

    // Initialize UART
    uart_init(UART_BAUDRATE, 0);

    // Loop to ask login / pass
    while (1) {
        // Ask username and password
        ask_username((char **)&in_username);
        // ask_password((char **)&in_password);
        // // Compare input username and password with stored ones
        // if (strcmp(in_username, username) == 0 \
        //     && strcmp(in_password, password) == 0) {
        //     welcome();
        //     break;
        // }
        // else {
        //     display_error();
        // }
    }

    // Loop if good login / pass
    while (1) {
        // Make the leds blink
    }


    return (0);
}
