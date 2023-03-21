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
// Timers
#define MAX_VAL_0 0
#define MAX_VAL_1 0
#define MAX_VAL_2 0
// RGB
#define DEL_RED PD5
#define DEL_GREEN PD6
#define DEL_BLUE PD3
// Program specific
#define BUFFER_SIZE 7
#define BACKSPACE 127
#define NEWLINE '\r'

/*
** -----------------------------------------------------------------------------
** Globals
** -----------------------------------------------------------------------------
*/
volatile const uint32_t         leds[3] = {DEL_RED, DEL_GREEN, DEL_BLUE};
volatile uint8_t                color_is_entered = 0;
volatile static unsigned char   color[BUFFER_SIZE] = "#000000";

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
** Helper Functions
** -----------------------------------------------------------------------------
*/
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

uint32_t    hexstr_to_int(volatile unsigned char *str){
	/*
    ** Convert string in hexa to int
    */

    uint32_t    res = 0;
    uint32_t    base = 1;

	for (uint8_t i = BUFFER_SIZE - 1; i > 0; i--){
        uart_tx(str[i]);
        if (str[i] >= '0' && str[i] <= '9') {
            res += (str[i] - 48) * base;
            base *= 16;
        }
        else if (str[i] >= 'A' && str[i] <= 'F') {
            res += (str[i] - 55) * base;
            base *= 16;
        }
        else {
            return (0xFFFFFFFF); // big number = error
        }
    }
	return (res);
}

/*
** -----------------------------------------------------------------------------
** Timers
** -----------------------------------------------------------------------------
*/
void    timer_0_conf() {
    /*
    ** This function configures the timer 0.
    **
    ** The timer should produce 2 interrupts by second :
    ** - The first one to turn the light on
    ** - The second to turn the light off
    */

    // 1. Waveform Generation Mode: Fast PWM
    TCCR0A |= (3 << WGM00);
    // 2. Value to compare the timer with (init 0)
    OCR0A = MAX_VAL_0;
    // 3. Interrupts when compare match and overflow
    TIMSK0 |= (1 << OCIE0A) | (1 << TOIE0);
    // 4. Clock prescale factor (1024) + launch the timer
    TCCR0B |= (1 << CS02) | (1 << CS00);
}

void    timer_1_conf(void) {
    /*
    ** This function configures the timer 1.
    **
    ** It should be able to produce 2 interrupts per second on the right moments
    ** 
    ** in an entire cycle (1 sec) :
    ** - 0 = off the entire second
    ** - 50 = on 50/255 = 0.19 seconds
    ** - 138 = on 138/255 = 0.54 seconds
    ** - 255 = on the entire second
    */

    // 2. Waveform Generation Mode: Fast PWM (Pulse With Modulation) with IRC1
    // as TOP
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM12) | (1 << WGM13);
    // 2bis. Use ICR1 as max value (255 here)
    ICR1 = 255;
    // 3. Interrupts when compare match and overflow
    TIMSK1 |= (1 << OCIE1A) | (1 << TOIE1);
    // 4. Value to compare the timer with
    OCR1A = MAX_VAL_1;
    // 5. Clock prescale factor (256) + launch the timer
    TCCR1B |= (1 << CS12);
}

void    timer_2_conf() {
    /*
    ** This function configures the timer 2.
    **
    ** The timer should produce 2 interrupts by second :
    ** - The first one to turn the light on
    ** - The second to turn the light off
    */

    // 1. Waveform Generation Mode: Fast PWM
    TCCR2A |= (3 << WGM20);
    // 2. Value to compare the timer with
    OCR2A = MAX_VAL_2;
    // 3. Interrupts when compare match and overflow
    TIMSK2 |= (1 << OCIE2A) | (1 << TOIE2);
    // 4. Clock prescale factor (1024) + launch the timer
    TCCR2B |= (1 << CS22) | (1 << CS20);
}

/*
** -----------------------------------------------------------------------------
** Interrupts
** -----------------------------------------------------------------------------
*/
ISR(TIMER0_COMPA_vect){
    PORTD &= ~(1 << DEL_RED);
}

ISR(TIMER0_OVF_vect){
    PORTD |= (1 << DEL_RED);
}

ISR(TIMER1_COMPA_vect) {
    PORTD &= ~(1 << DEL_GREEN);
}

ISR(TIMER1_OVF_vect){
    PORTD |= (1 << DEL_GREEN);
}

ISR(TIMER2_COMPA_vect){
    PORTD &= ~(1 << DEL_BLUE);
}

ISR(TIMER2_OVF_vect){
    PORTD |= (1 << DEL_BLUE);
}

ISR(USART_RX_vect){
    unsigned char   c;
    static uint8_t  rx_write_pos = 0;

    // Read char from buffer    
    c = UDR0;

    // Save char into the proper buffer and display it
    if (c == BACKSPACE) {
        if (rx_write_pos != 0) {
            --rx_write_pos;
            color[rx_write_pos] = '\0';
            uart_printstr("\b \b");
        }
    }
    else if (c == NEWLINE) {
        color_is_entered = 1;
        rx_write_pos = 0;
        uart_printstr("\r\n");
    }
    else {
        color[rx_write_pos] = c;
        uart_tx(c);
        ++rx_write_pos;
    }
    
    // Overflow management : restart to pos 0
    if (rx_write_pos >= BUFFER_SIZE) {
        rx_write_pos = 0;
    }
}

/*
** -----------------------------------------------------------------------------
** RGD LED
** -----------------------------------------------------------------------------
*/

void    set_rgb(uint8_t r, uint8_t g, uint8_t b){
    /*
    ** Convert a rgb color on 24 bits (000000 > FFFFFF) to a led response
    */

    // RED
    OCR0A = r;
    // GREEN
    OCR1A = g;
    // BLUE
    OCR2A = b;
}

void    wheel(uint8_t pos) {
    /*
    ** Rotate the colors of the led changing the pos on color wheel
    */

    pos = 255 - pos;
    if (pos < 85) {
        set_rgb(255 - pos * 3, 0, pos * 3);
    } else if (pos < 170) {
        pos = pos - 85;
        set_rgb(0, pos * 3, 255 - pos * 3);
    } else {
        pos = pos - 170;
        set_rgb(pos * 3, 255 - pos * 3, 0);
    }
}

void    init_rgb(void){
    /*
    ** Set the led as output and turn them off
    */

    // Initialize the LEDS    
    const uint8_t   nb_rgb = sizeof(leds) / sizeof(leds[0]);

    for (uint8_t i = 0; i < nb_rgb; i++) {
        DDRD |= (1 << leds[i]); // Output
        PORTD &= ~(1 << leds[i]); // Off by default
    }

    // Initialize the timers
    timer_0_conf();
    timer_1_conf();
    timer_2_conf();
}

/*
** -----------------------------------------------------------------------------
** Program helpers
** -----------------------------------------------------------------------------
*/
void    ask_color() {
    uart_printstr("Enter a color to display: ");
    while (color_is_entered == 0)
        ;
    color_is_entered = 0;
}

void    display_error(){
    uart_printstr("Bad color format (#000000 to #FFFFFF expected)\n\n\r");
}

uint32_t    get_color(){

    // If the color does not start with # > Error
    if (color[0] && color[0] != '#') { return (0xFFFFFFFF); }
    if (ft_strlen(color) < 7) { return (0xFFFFFFFF); }

    return hexstr_to_int(color);
}

/*
** -----------------------------------------------------------------------------
** MAIN program
** -----------------------------------------------------------------------------
*/
int main(void){

    uint32_t int_color = 0;

    // Initialize UART
    uart_init(UART_BAUDRATE, 0);

    // Init RGB leds + timers
    init_rgb();

    // Enable global interrupts
    SREG |= (1 << 7);

    // Loop to enter colors using uart
    while (1) {
        // Ask a color
        ask_color();
        // Compare input username and password with stored ones
        int_color = get_color();
        if (int_color != 0xFFFFFFFF){ // very basic check, to complete
            set_rgb((uint8_t)((int_color & 0x00FF0000) >> 16), \
                    (uint8_t)((int_color & 0x0000FF00) >> 8), \
                    (uint8_t)(int_color & 0x000000FF));
        } else {
            display_error();
        }
    }

    return (0);
}
