// AVR
#include <avr/io.h>
#ifndef F_CPU
# define F_CPU 16000000UL
#endif
#define UART_BAUDRATE 115200
#define __INTR_ATTRS used, externally_visible
#define ISR(vector, ...) \
        void vector (void) __attribute__ ((signal,__INTR_ATTRS)) __VA_ARGS__; \
        void vector (void)
// Program specific
#define BUFFER_SIZE 32
#define BACKSPACE 127
#define NEWLINE '\r'
#define USERNAME 0
#define PASSWORD 1

// Globals
volatile static unsigned char   username[BUFFER_SIZE] = "twagner";
volatile static unsigned char   password[BUFFER_SIZE] = "password";
volatile static unsigned char   *rx_buffer[2];
volatile static unsigned char   buffer_username[BUFFER_SIZE];
volatile static unsigned char   buffer_password[BUFFER_SIZE];
volatile static uint8_t         curr_buffer = USERNAME;

const uint8_t                   leds[] = {PB0, PB1, PB2, PB4};
const uint8_t                   nb_leds = sizeof(leds) / sizeof(leds[0]);

// Helper functions
int	ft_strcmp(const volatile unsigned char *s1, \
              const volatile unsigned char *s2)
{
	while (*s1 || *s2)
	{
		if (*s1 != *s2)
			return (*s1 - *s2);
		s1++;
		s2++;
	}
	return (0);
}

void	ft_bzero(volatile void *s, uint32_t n)
{
	uint32_t    i;

	i = -1;
	while (++i < n)
	{	
		*((unsigned char *)s) = 0;
		++s;
	}
}

int	ft_strlen(const volatile unsigned char *s)
{
	int	i;

	if (!s)
		return (0);
	i = 0;
	while (*(s + i))
		++i;
    return (i);
}

uint32_t    my_log2(uint32_t n) {
    int result = 0;

    n = n / 2;
    while (n) {
        ++result;
        n = n / 2;
    }
    return result;
}

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

// Interrupts
ISR(USART_RX_vect){
    unsigned char   c;
    uint32_t        buf_len;
    static uint8_t  rx_write_pos = 0;

    // Read char from buffer    
    c = UDR0;

    // Save char into the proper buffer and display it
    if (c == BACKSPACE) {
        if (rx_write_pos != 0) {
            --rx_write_pos;
            rx_buffer[curr_buffer][rx_write_pos] = '\0';
            uart_printstr("\r\033[2K");
            if (curr_buffer == USERNAME) {
                uart_printstr("\tusername:");
                uart_printstr(rx_buffer[curr_buffer]);
            }
            else {
                uart_printstr("\tpassword:");
                buf_len = ft_strlen(rx_buffer[curr_buffer]);
                for (uint32_t i = 0; i < buf_len; i++) {
                    uart_tx('*');
                }
            }
        }
    }
    else if (c == NEWLINE) {
        curr_buffer ^= 1;
        rx_write_pos = 0;
        uart_printstr("\r\n");
    }
    else {
        rx_buffer[curr_buffer][rx_write_pos] = c;
        if (curr_buffer == PASSWORD) {
            uart_tx('*');
        }
        else {
            uart_tx(c);
        }
        ++rx_write_pos;
    }
    
    // Overflow management : restart to pos 0
    if (rx_write_pos >= BUFFER_SIZE) {
        rx_write_pos = 0;
    }
}

// Interrupt Service Routine of timer 0 that triggers on compare match
ISR(TIMER1_COMPA_vect)
{
    /*
    ** This is triggered every time timer1 comp A match
    */

    // Transmit hello world!
    for (uint8_t i = 0; i < nb_leds; i++) {
        // Toogle lights
        PORTB ^= (1 << leds[i]);      
    }
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

// Program helpers
void    ask_username() {
    ft_bzero(buffer_username, BUFFER_SIZE);
    uart_printstr("Enter your login:\n\r");
    uart_printstr("\tusername:");
    while (curr_buffer == USERNAME)
        ;
}

void    ask_password() {
    ft_bzero(buffer_password, BUFFER_SIZE);
    uart_printstr("\tpassword:");
    while (curr_buffer == PASSWORD)
        ;
}

void    welcome(){
    uart_printstr("Hello spectre!\n\r");
    uart_printstr("Shall we play a game?\n\r");
}

void    display_error(){
    uart_printstr("Bad combinaison username/password\n\n\r");
}

// Main
int main(void){
    
    rx_buffer[USERNAME] = (unsigned char *)&buffer_username;
    rx_buffer[PASSWORD] = (unsigned char *)&buffer_password;

    // Initialize UART
    uart_init(UART_BAUDRATE, 0);

    // Loop to ask login / pass
    while (1) {
        // Ask username and password
        ask_username();
        ask_password();
        // Compare input username and password with stored ones
        if (ft_strcmp(rx_buffer[USERNAME], username) == 0 \
            && ft_strcmp(rx_buffer[PASSWORD], password) == 0) {
            welcome();
            break;
        }
        else {
            display_error();
        }
    }

    // Del init
    for (uint8_t i = 0; i < nb_leds; i++) {
        DDRB |= (1 << leds[i]); // Outputs
        PORTB &= ~(1 << leds[i]); // Init
    }

    // Conf timer 1
    timer_1_conf(256, 4);

    // Reward loop
    while (1) {
        
    }

    return (0);
}
