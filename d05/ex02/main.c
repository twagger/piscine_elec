// AVR
#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdbool.h>
#ifndef F_CPU
# define F_CPU 16000000UL
#endif
// Switch
#define DEBOUNCING 250 // debouncing delay (in ms)
// EEPROM
#define ADDRESS 0
#define NB_COUNTERS 4
#define MAGIC 0xA5
// UART
#define UART_BAUDRATE 115200
#define __INTR_ATTRS used, externally_visible
#define ISR(vector, ...)            \
        void vector (void) __attribute__ ((signal,__INTR_ATTRS)) __VA_ARGS__; \
        void vector (void)

/*
** -----------------------------------------------------------------------------
** Globals
** -----------------------------------------------------------------------------
*/
volatile uint8_t        counter[NB_COUNTERS];
volatile uint8_t        selected = 0;
volatile const uint8_t  leds[] = {PB0, PB1, PB2, PB4};

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
void    display_bin(uint8_t number){
    /*
    ** Display the 4 LSB of a number using 4 leds of the board
    */

    const uint8_t   nb_leds = sizeof(leds) / sizeof(leds[0]);

    for (uint8_t i = 0; i < nb_leds; i++) {
        if (number & 1 << i){
            PORTB |= (1 << leds[i]);
        }
        else { PORTB &= ~(1 << leds[i]); }
    }
}

void    int0_init(void){
    /*
    ** Enable and conf interruptions for SW1
    */

    // Parameterization of the interrupt
    // 1. Sense control : low level
    // As bouncing will generates more edges than level, it is safer to chose
    // the low level instead of the falling or rising edge events.
    EICRA &= ~((1 << ISC00) | (1 << ISC01));
    // 2. Enable INT0 interrupt
    EIMSK |= (1 << INT0);
    // 3. Enable global interrupt by setting global interrupt enable bit to 1
    SREG |= (1 << 7); // sei(); is also valid
}

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

void    counters_setup(void){
    // Read counters current state
    for (uint8_t i = 0; i < NB_COUNTERS; i++){
        counter[i] = eeprom_read_byte((uint8_t *)(0 + i));
    }
    // Display binary value on leds for currently selected counter
    display_bin(counter[selected]);
    // This enables to reset the counter with two consecutive resets
    for (uint8_t i = 0; i < NB_COUNTERS; i++){
        eeprom_write_byte((uint8_t *)(0 + i), 0);
    }
}

int	ft_strncmp(const char *s1, const char *s2, size_t n){
	size_t	i;

	i = 0;
	while ((s1[i] || s2[i]) && i < n)
	{
		if ((unsigned char)s1[i] != (unsigned char)s2[i])
			return ((unsigned char)s1[i] - (unsigned char)s2[i]);
		++i;
	}
	return (0);
}

/*
** -----------------------------------------------------------------------------
** Interrupts
** -----------------------------------------------------------------------------
*/
ISR(INT0_vect) {
    /*
    ** ISR of SW1
    */

    // Here I don't read counter because it has been loaded by counter_setup()
    counter[selected]++;
    uart_byte_printer(counter[selected]);
    eeprom_write_byte((uint8_t *)(0 + selected), counter[selected]);
    display_bin(counter[selected]);
    _delay_ms(DEBOUNCING);
    EIFR = (1 << INTF0); // clear external interrupt flag 
}

ISR(PCINT2_vect){
    /*
    ** This function will change the counter a counter when it is triggered
    */

    if ((PIND & (1 << PD4)) == 0) { // I use PINX to be clean (could use global)
        selected = (selected + 1 >= NB_COUNTERS) ? 0 : selected + 1;
        display_bin(counter[selected]);
        _delay_ms(DEBOUNCING);
        EIFR = (1 << INTF0); // clear external interrupt flag 
    }
}

/*
** -----------------------------------------------------------------------------
** EEPROM functions
** -----------------------------------------------------------------------------
*/

bool    is_my_magic(size_t offset, uint8_t my_magic){
    /*
    ** Checks if a memory block is prefixed by my nagic number
    */

    uint8_t to_check_magic;

    // Get the magic number
    if (offset <= 0) { return (0); }
    to_check_magic = eeprom_read_byte((uint8_t *)(offset - 1));
    if (to_check_magic == my_magic){
        return (1);
    }
    else { return (0); }
}

bool    safe_eeprom_read(void *buffer, size_t offset, size_t length){
    /*
    ** read data IF they have been written by me (check with my magic number)
    ** 
    ** - buffer : the destination of the readed data
    ** - offset : the memory offset (starting from 0)
    ** - length : the length of the data to read
    */

    if (is_my_magic(offset, MAGIC)){
        // The read is safe, load the buffer
        eeprom_read_block((void *)buffer, (const void *)offset, length);
        return (1);
    }
    else {
        // The read is unsafe
        return (0);
    }

}

bool    safe_eeprom_write(void *buffer, size_t offset, size_t length){
    /*
    ** write data with a magic number to identify them as written by me (or do
    ** not re-write them if it is already written by me)
    ** 
    ** - buffer : the data to write
    ** - offset : the memory offset (starting from 0)
    ** - length : the length of the data to write
    */
    
    void    *tmp_buffer = NULL;

    // If I cannot add magic for the address
    if (offset <= 0){
        return (0);
    }

    // Checks if read is safe
    if (safe_eeprom_read(tmp_buffer, offset, length)) {
        // Rewrite only if data is different
        if (ft_strncmp(tmp_buffer, buffer, length) != 0) {
            eeprom_write_block((const void *)buffer, (void *)(offset), length);
        }
    }
    else {
        // The data are not writtend by me : I respect that and I leave
        return (0);
    }
    return (1);
}

/*
** -----------------------------------------------------------------------------
** MAIN program
** -----------------------------------------------------------------------------
*/
int main(void){

    // uint8_t sw3_status;

    // Initialize UART
    uart_init(UART_BAUDRATE, 0);

    // Init SW1 interrupt + SW2 interrupt + enable global interrupts
    int0_init();
    pcint20_init();
    SREG |= (1 << 7);

    // Setup counter
    counters_setup();

    display_bin(counter[selected]);

    while (1) {
    }

    return (0);
}
