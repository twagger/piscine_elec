// AVR
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/boot.h>
#ifndef F_CPU
# define F_CPU 16000000UL
#endif
// UART
#define UART_BAUDRATE 115200
#define __INTR_ATTRS used, externally_visible
#define ISR(vector, ...)            \
        void vector (void) __attribute__ ((signal,__INTR_ATTRS)) __VA_ARGS__; \
        void vector (void)
// Program specific
#define RV1 0
#define LDR 1
#define NTC 2
// Temperature calibration addresses
#define TS_OFFSET 0x0002
#define TS_GAIN 0x0003

/*
** -----------------------------------------------------------------------------
** Globals
** -----------------------------------------------------------------------------
*/
volatile static uint16_t    value = 0;
volatile static uint8_t     current_conv = RV1;


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
    // ADMUX |= (1 << ADLAR); // Adjust right so we just read ADC for 10bits

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

void    adc_temp_sensor(void){
    /*
    ** Configure and initialize ADC (Analog to Digital Converter)
    ** 
    ** All configs are explicits here for learning purposes (even if I don't 
    ** need to write a 0 if it is the default value).
    */

    // Enable temperature sensor (this overwrite previous params on ADMUX)
    ADMUX = (1 << MUX3);

    // Reference Voltage (internal 1.1v)
    ADMUX |= (3 << REFS0);
}

void    start_conversion(void){
    /*
    ** Explicitly start an ADC conversion for Temperature
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
    char    result[4] = {'0','0','0', 0};
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
** Interrupts
** -----------------------------------------------------------------------------
*/
ISR(TIMER1_COMPA_vect)
{
    /*
    ** This is triggered every time timer1 comp A match
    */

    // Display values
    for (uint8_t i = 0; i < 3; i++) {
        display_byte_decimal(value);
        uart_printstr("\n\r");
    }
}

ISR(ADC_vect)
{
    /*
    ** ADC conversion complete
    */
    // uint8_t ts_gain = boot_signature_byte_get(TS_GAIN);
    // uint8_t ts_offset = boot_signature_byte_get(TS_OFFSET);

    // Read the value of ADC register and update global variable with it
    value = ADC;
    // Compute the value to get a temperature in Celsius
    // I use this formula because I didn't find any correct documentation on how
    // to properly use TS_GAIN an TS_OFFSET
    value = (value - 324.31) / 1.22;

    // Debug
    // uart_byte_printer(ts_gain);
    // uart_byte_printer(ts_offset);
    // uart_byte_printer(value & 0xFF);
    // uart_byte_printer((value >> 8) & 0xFF);
    _delay_ms(1000);


    // Clear the ADIF flag with logical 1 to disable pending interrupts
    ADCSRA |= (1 << ADIF);
    start_conversion();
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
    adc_temp_sensor();
    start_conversion();

    // Configure Timer 1 to interrupt every 20ms (50Hz frequency)
    timer_1_conf(256, 50);

    // Loop
    while (1) {

    }

    return (0);
}
