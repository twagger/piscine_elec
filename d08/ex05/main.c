// AVR
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#ifndef F_CPU
# define F_CPU 16000000UL
#endif
// Switch
#define DEBOUNCING 250 // debouncing delay (in ms)
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
// SPI – Serial Peripheral Interface
#define MOSI PB3
#define MISO PB4
#define SCK PB5
#define SS PB2
// Colors
#define RED 0
#define GREEN 1
#define BLUE 2
// SPI LEDS
#define D6 0
#define D7 1
#define D8 2
#define BRIGHTNESS 0x01
// I2C
#define EXPANDER_W 0b01000000
#define EXPANDER_R 0b01000001
#define OPT_DATA 0
#define OPT_START 1
#define OPT_STOP 2

/*
** -----------------------------------------------------------------------------
** Globals
** -----------------------------------------------------------------------------
*/
volatile static uint8_t     position = 0;
volatile static uint8_t     rgb[] = {0, 0, 0};
volatile static uint32_t    colors[] = {0, 0, 0};
volatile static uint8_t     selected_rgb = RED;
volatile uint8_t            selected_led = 0;
// I2C
static uint8_t          i2c_error = 0;
static const uint8_t    debug = 0;
static const char       *status_mess[] = {\
                      "A START condition has been transmitted", \
                      "SLA+W has been transmitted; ACK received", \
                      "SLA+R has been transmitted; ACK received", \
                      "Data byte has been transmitted; ACK received", \
                      "A repeated START condition has been transmitted", \
                      "Data byte has been received; NOT ACK has been returned"};
static const uint8_t    status_codes[] = {TW_START, TW_MT_SLA_ACK,
                                          TW_MR_SLA_ACK, TW_MT_DATA_ACK,
                                          TW_REP_START, TW_MR_DATA_NACK};

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
** I2C Functions
** -----------------------------------------------------------------------------
*/
void    wait_for_transmission(void) {
    /*
    ** Waits for transmission to be ok by checking TWINT bit in Control registry
    */

    while (!(TWCR & (1 << TWINT)))
        ;
}

int check_for_status(uint8_t expected){
    /*
    ** Checks for I2C status in TWSR registry
    */

    if ((TWSR & 0xF8) != expected){
        // push error to screen
        uart_printstr("Error");
        uart_byte_printer((TWSR & 0xF8));
        uart_byte_printer(expected);
        i2c_error = 1;
        return (1);
    }
    else if (debug == 1) {
        for (uint8_t i = 0; i < sizeof(status_codes) / sizeof(status_codes[0]);\
             i++) {
            if (expected == status_codes[i]) {
                uart_printstr((const char *)status_mess[i]);
                uart_printstr("\n\r");
            }
        }
    }
    return (0);
}

void    send_to_i2c(uint8_t option, unsigned char data, uint8_t expected){
    /*
    ** Send to i2c bus
    */

    // START / STOP / RESTART
    if (option == OPT_START) {
        TWCR = (1 << TWINT) | (1 << TWSTA)| (1 << TWEN);
        wait_for_transmission();
        if (check_for_status(expected) == 1) {
            return;
        }
    }
    else if (option == OPT_STOP) {
        TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
        _delay_ms(0.01); // to be sure the bus is idle for the next start
    }
    else if (option == OPT_DATA) {
        // DATA
        // Load data register with data
        TWDR = data;
        // Transmit
        TWCR = (1 << TWINT) | (1 << TWEN);
        wait_for_transmission();
        if (check_for_status(expected) == 1) {
            return;
        }
    }
}

void    i2c_init(void){
    /*
    ** Initializes I2C on the MCU.
    **
    ** - Communication frequency : 100kHz
    */

    // Configure prescale and bit rate for 100kHz
    TWSR &= ~((1 << TWPS0) | (1 << TWPS1));
    TWBR = 72;
}

void    i2c_start(uint8_t data, uint8_t expected){
    /*
    ** Starts an I2C transmission between the MCU and the sensor. Prepare it in 
    ** write mode. The bus is then considered busy for others.
    */

    // Send start condition
    send_to_i2c(OPT_START, 0, TW_START);
    if (i2c_error) { return; }
        
    // Send slave address
    send_to_i2c(OPT_DATA, data, expected);
}

void    i2c_repeat_start(uint8_t data, uint8_t expected){
    /*
    ** Starts an I2C transmission between the MCU and the sensor. Prepare it in 
    ** write mode. The bus is then considered busy for others.
    */

    // Send start condition
    send_to_i2c(OPT_START, 0, TW_REP_START);
    if (i2c_error) { return; }
        
    // Send slave address
    send_to_i2c(OPT_DATA, data, expected);
}

void    i2c_write(unsigned char data){
    /*
    ** Write a byte to I2C bus
    */

    send_to_i2c(OPT_DATA, data, TW_MT_DATA_ACK);
}

uint8_t i2c_read(void){
    /*
    ** Display the content of TWDR after the sensor measurement
    */

    wait_for_transmission();
    return TWDR;
}

void    i2c_stop(void){
    /*
    ** Stops the I2C communication between the MCU and the sensor. The bus is no
    ** longer busy.
    */

    send_to_i2c(OPT_STOP, 0, 0);
    if (debug) {
        uart_printstr("A STOP condition has been transmitted\n\r\n\r");
    }
}

void    send_nack(void){
    TWCR = (1 << TWINT) | (1 << TWEN);
    wait_for_transmission();
    if (check_for_status(TW_MR_DATA_NACK) == 1) {
        return;
    }
}

/*
** -----------------------------------------------------------------------------
** I2C Expander Functions
** -----------------------------------------------------------------------------
*/
void    init_i2e_expander(void){
    /*
    ** Initializes i2c expander
    */

    // Start transmission with the E2C IO expander
    i2c_start(EXPANDER_W, TW_MT_SLA_ACK);
    if (i2c_error) { return; }

    // Configure SW3 as input and DELS as output
    i2c_write(6); // configuration port 0
    if (i2c_error) { return; }

    // Configuration data
    i2c_write(1); // SW3 input, LEDS output
    if (i2c_error) { return; }

    // Stop i2c
    i2c_stop();
}

/*
** -----------------------------------------------------------------------------
** I2C SW3 Functions
** -----------------------------------------------------------------------------
*/
uint8_t check_sw3(void){
    /*
    ** Reads from E2C IO expander to check if SW3 is pressed
    */

    uint8_t reg_value;

    // Start in Write mode to specify what we want to read
    i2c_start(EXPANDER_W, TW_MT_SLA_ACK);
    if (i2c_error) { return (2); }

    // Send register address that we want to read
    i2c_write(0); // Input port register
    if (i2c_error) { return (2); }

    // Stop i2c
    // i2c_stop();

    // Start in read mode to read the data from register
    i2c_repeat_start(EXPANDER_R, TW_MR_SLA_ACK);
    if (i2c_error) { return (2); }

    // Send NACK to stop after one byte sent from I2C expander
    send_nack();
    if (i2c_error) { return (2); }

    // Read the value of register
    reg_value = i2c_read();

    // Stop i2c
    i2c_stop();

    return (!(reg_value & 1));
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

    // display color with d6
    spi_write((uint8_t)(7 << 5) | (brightness & 0x1F));
    spi_write((uint8_t)(color & 0xFF));
    spi_write((uint8_t)(color >> 8) & 0xFF);
    spi_write((uint8_t)(color >> 16) & 0xFF);
}

/*
** -----------------------------------------------------------------------------
** Program helpers
** -----------------------------------------------------------------------------
*/
uint32_t    get_updated_color(void){
    uint32_t    color = 0;

    // Rebuild color from rgb
    color = rgb[RED];
    color <<= 8;
    color |= rgb[GREEN];
    color <<= 8;
    color |= rgb[BLUE];
    return color;
}

void    display_colors_on_leds(){
       
    spi_apa102_start_frame();

    spi_led(colors[D6], BRIGHTNESS);
    spi_led(colors[D7], BRIGHTNESS);
    spi_led(colors[D8], BRIGHTNESS);

    spi_apa102_end_frame();
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

/*
** -----------------------------------------------------------------------------
** Interrupts
** -----------------------------------------------------------------------------
*/
void    pcint20_init() {
    /*
    ** Parameterization of the PCINT20 interrupt
    ** Carefull : ANY change will trigger the associated ISR (no sense control)
    */

    // Enable PCINT23:16 range interrupts
    PCICR |= (1 << PCIE2);
    // Enable PCINT20 interrupts only
    PCMSK2 = (1 << PCINT20);
}

ISR(TIMER1_COMPA_vect){
    /*
    ** Timer to refresh the leds
    */

    // Update the current led with adjusted RGB
    colors[selected_led] = get_updated_color();
    // Refresh leds color
    display_colors_on_leds();
}

ISR(ADC_vect){
    /*
    ** ADC conversion complete
    */

    // Read the value of ADC register and update global variable with it
    position = ADCH;
    // Change the value of the selected primary color
    rgb[selected_rgb] = position;
    // Clear the ADIF flag with logical 1 to disable pending interrupts
    ADCSRA |= (1 << ADIF);
    // Start a new conversion
    start_conversion();
}

ISR(PCINT2_vect){
    /*
    ** Interruption when SW2 is pressed
    */

    if ((PIND & (1 << PD4)) == 0) { // I use PINX to be clean (could use global)
        selected_rgb = (selected_rgb + 1 > 2) ? 0 : selected_rgb + 1;
        _delay_ms(DEBOUNCING);
        EIFR = (1 << INTF0); // clear external interrupt flag
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

    // Initialize SPI as Master
    spi_master_init();

    // Init SW2 interrupt
    pcint20_init();

    // Init SW3 I2C
    i2c_init();
    init_i2e_expander();

    // Start a timer to refresh the leds every 20ms
    timer_1_conf(256, 50);

    // Loop
    while (1) {
        if (check_sw3() == 1){ // SW3 is pressed
            selected_led = (selected_led + 1 > 2) ? 0 : selected_led + 1;
            selected_rgb = RED; // We restart from red
            _delay_ms(DEBOUNCING);
        }
    }

    return (0);
}
