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
#define OFF 0x000000
// Program specific
#define BUFFER_SIZE 13
#define BACKSPACE 127
#define NEWLINE '\r'
// SPI LEDS
#define D6 0
#define D7 1
#define D8 2
#define BRIGHTNESS 0x01

/*
** -----------------------------------------------------------------------------
** Globals
** -----------------------------------------------------------------------------
*/
volatile uint8_t                selected_led = 0;
volatile uint32_t               selected_color = 0;
volatile uint8_t                rainbow = 0;
volatile uint8_t                position = 254;
volatile uint8_t                color_is_entered = 0;
volatile static unsigned char   color[BUFFER_SIZE] = {0};

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

void	ft_bzero(volatile void *s, uint32_t n){
	uint32_t    i;

	i = -1;
	while (++i < n)
	{	
		*((unsigned char *)s) = 0;
		++s;
	}
}

int	ft_strcmp(const volatile unsigned char *s1, const char *s2){
	while (*s1 || *s2)
	{
		if (*s1 != *s2)
			return (*s1 - *s2);
		s1++;
		s2++;
	}
	return (0);
}

int	ft_strlen(const volatile unsigned char *s){
	int	i;

	if (!s)
		return (0);
	i = 0;
	while (*(s + i))
		++i;
    return (i);
}

uint32_t    hexstr_to_int(volatile unsigned char *str, uint8_t size){
	/*
    ** Convert string in hexa to int.
    **
    ** The string should be shaped like "#000000"
    ** The size param ignore the first '#'
    */

    uint32_t    res = 0;
    uint32_t    base = 1;

	for (uint8_t i = size; i > 0; i--){
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
** Timers
** -----------------------------------------------------------------------------
*/
void    timer_1_conf(uint16_t prescale, uint8_t frequency) {
    /*
    ** Produce an interrupt according to a frequency
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

void    stop_timer_1(void) {
    /*
    ** Stops the timer 1
    */

    TCCR1B &= ~((1 << CS10) | (1 << CS11) | (1 << CS12));
}


/*
** -----------------------------------------------------------------------------
** Program helpers
** -----------------------------------------------------------------------------
*/
// SPI
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

void    spi_led(uint32_t color, uint8_t brightness){
    /*
    ** Turn on RGB led with the specified color and brightness
    ** 
    ** The frame should be started before and ended after this
    */

    // display color
    spi_write((uint8_t)(7 << 5) | (brightness & 0x1F));
    spi_write((uint8_t)(color & 0xFF));
    spi_write((uint8_t)((color >> 8) & 0xFF));
    spi_write((uint8_t)((color >> 16) & 0xFF));
}

void    display_color_on_led(){
    /*
    ** Display a certain color on a certain led
    */

    // Stops timer 1
    stop_timer_1();

    // Send colors to leds
    spi_apa102_start_frame();
    spi_led((selected_led == 0) ? selected_color : OFF, BRIGHTNESS);
    spi_led((selected_led == 1) ? selected_color : OFF, BRIGHTNESS);
    spi_led((selected_led == 2) ? selected_color : OFF, BRIGHTNESS);
    spi_apa102_end_frame();
}

void    display_rainbow(){
    /*
    ** Display a color wheel on all leds
    */
    
    // Enable timer 1
    timer_1_conf(256, 1);
}

// UART
void    ask_color() {
    ft_bzero(color, BUFFER_SIZE);
    uart_printstr("Enter a color to display and a led (#000000DX): ");
    while (color_is_entered == 0)
        ;
    color_is_entered = 0;
}

void    display_usage(void) {
    uart_printstr("\n\rUsage :\n\r");
    uart_printstr("Enter a color like : #FF000D6 to display red on D6\n\r");
    uart_printstr("Available DELS : D6, D7, D8\n\r");
    uart_printstr("Option : #FULLRAINBOW, shows color wheel on leds\n\r\n\r");

}

uint8_t is_color_valid(void){

    uint32_t conv_color;

    // Basic checks
    if (color[0] && color[0] != '#') { return (0); }

    // Check if rainbow
    if (ft_strcmp(color, "#FULLRAINBOW") == 0) {
        rainbow = 1;
        return (1);
    } else {
        rainbow = 0;
        // Basic check
        if (ft_strlen(color) != 9) { return (0); }
        // Conversion of color bits
        conv_color = hexstr_to_int(color, 6);
        if (conv_color == 0xFFFFFFFF) { return (0); }
        else { selected_color = conv_color; }
        // Get the led
        if (ft_strcmp(color + 7, "D6") == 0) { selected_led = D6; }
        else if (ft_strcmp(color + 7, "D7") == 0) { selected_led = D7; }
        else if (ft_strcmp(color + 7, "D8") == 0) { selected_led = D8; }
        else { return (0); }
        return (1);
    }
    return (0);
}

void    wheel(uint8_t pos) {
    /*
    ** Rotate the colors of the led changing the pos on color wheel
    */
    uint32_t    wheel_color = 0;

    pos = 255 - pos;
    if (pos < 85) {
        wheel_color = (255 - pos * 3);
        wheel_color <<= 16;
        wheel_color |= (pos * 3);
    } else if (pos < 170) {
        pos = pos - 85;
        wheel_color = (pos * 3);
        wheel_color <<= 8;
        wheel_color |= (255 - pos * 3);
    } else {
        pos = pos - 170;
        wheel_color = (pos * 3);
        wheel_color <<= 8;
        wheel_color |= (255 - pos * 3);
        wheel_color <<= 8;
    }

    // Display color on all leds
    spi_apa102_start_frame();
    spi_led(wheel_color, BRIGHTNESS);
    spi_led(wheel_color, BRIGHTNESS);
    spi_led(wheel_color, BRIGHTNESS);
    spi_apa102_end_frame();
}

/*
** -----------------------------------------------------------------------------
** Interrupts
** -----------------------------------------------------------------------------
*/

ISR(TIMER1_COMPA_vect){
    /*
    ** This is triggered on every compare match on timer 1
    */

    // Display wheel
    wheel(position);
    // Move to next position on the wheel
    position = (position - 1 < 0) ? 255 : position - 1; 
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

    // Loop
    while (1) {
        ask_color();
        if (is_color_valid()) {
            if (rainbow) {
                display_rainbow();
            } else {
                display_color_on_led();
            }
        } else {
            display_usage();
        }
    }

    return (0);
}
