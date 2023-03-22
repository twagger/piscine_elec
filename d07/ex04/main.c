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
#define RV1 0
#define LDR 1
#define NTC 2

/*
** -----------------------------------------------------------------------------
** Globals
** -----------------------------------------------------------------------------
*/
volatile static uint8_t position = 0;
volatile const uint32_t rgb[] = {DEL_RED, DEL_GREEN, DEL_BLUE};
volatile const uint32_t led[] = {PB0, PB1, PB2, PB4};
const uint8_t           nb_leds = sizeof(led) / sizeof(led[0]);


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
    ADMUX |= (1 << ADLAR); // Adjust left so we just read ADC for 8bits

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
** Program helpers
** -----------------------------------------------------------------------------
*/
void    display_byte_hexa(uint16_t byte){
    /*
    ** Displays a byte in hexa using uart rx
    */

    char    *base = "0123456789abcdef";
    char    result[4] = {'0', '0', '0', 0};
    uint8_t i = 2;

    while (byte) {
        result[i] = base[byte % 16];
        byte /= 16;
        i--;
    }
    uart_printstr(result);
}

void    display_byte_decimal(uint16_t byte){
    /*
    ** Displays a byte in hexa using uart rx
    */

    char    *base = "0123456789";
    char    result[5] = {'0', '0', '0', '0', 0};
    uint8_t i = 3;

    while (byte) {
        result[i] = base[byte % 10];
        byte /= 10;
        i--;
    }
    uart_printstr(result);
}

/*
** -----------------------------------------------------------------------------
** Classic LEDS
** -----------------------------------------------------------------------------
*/
void    init_leds(void){
    
    for (uint8_t i = 0; i < nb_leds; i++) {
        DDRB |= (1 << led[i]); // Outputs
        PORTB &= ~(1 << led[i]); // Of by default
    }
}

void    display_level(uint8_t level){
    
    uint8_t percentage = ((float)level / 255.0) * 100;
    for (uint8_t i = 0; i < nb_leds; i++) {
        if (percentage >= (i + 1) * 25) {
            PORTB |= (1 << led[i]); // On
        } else {
            PORTB &= ~(1 << led[i]); // Off
        }
    }
}

/*
** -----------------------------------------------------------------------------
** Interrupts
** -----------------------------------------------------------------------------
*/

ISR(ADC_vect){
    /*
    ** ADC conversion complete
    */

    // Read the value of ADC register and update global variable with it
    position = ADCH;
    // Display level using leds (25% 50% 75% 100%)
    display_level(position);
    // Clear the ADIF flag with logical 1 to disable pending interrupts
    ADCSRA |= (1 << ADIF);
    // Start a new conversion
    start_conversion();
}

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

/*
** -----------------------------------------------------------------------------
** Timers
** -----------------------------------------------------------------------------
*/

void    timer_0_conf(uint16_t prescale) {
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
    // 4. Clock prescale factor
    // TCCR0B |= (1 << CS02) | (1 << CS00);
    TCCR0B |= ((uint32_t)(my_log2(prescale) / 2) << CS00);
}

void    timer_1_conf(uint16_t prescale) {
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
    // 5. Clock prescale factor
    // TCCR1B |= (1 << CS12);
    TCCR1B |= ((uint32_t)(my_log2(prescale) / 2) << CS10);
}

void    timer_2_conf(uint16_t prescale) {
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
    // 4. Clock prescale factor
    // TCCR2B |= (1 << CS22) | (1 << CS20);
    TCCR2B |= ((uint32_t)(my_log2(prescale) / 2) << CS20);
}

/*
** -----------------------------------------------------------------------------
** RGD LED
** -----------------------------------------------------------------------------
*/
void    init_rgb(void){
    /*
    ** Set the led as output and turn them off
    */

    // Initialize the RGB LEDS    
    const uint8_t   nb_rgb = sizeof(rgb) / sizeof(rgb[0]);

    for (uint8_t i = 0; i < nb_rgb; i++) {
        DDRD |= (1 << rgb[i]); // Output
        PORTD &= ~(1 << rgb[i]); // Off by default
    }

    // Initialize the timers
    timer_0_conf(1024);
    timer_1_conf(256);
    timer_2_conf(1024);
}

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

    // Initialize ADC and start the first conversion
    adc_init();
    start_conversion();

    // Init RGB led + timers
    init_rgb();

    // Init 4 classic leds
    init_leds();

    // Loop
    while (1) {
        wheel(position);
    }

    return (0);
}
