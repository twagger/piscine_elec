// AVR
#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdbool.h>
#ifndef F_CPU
# define F_CPU 16000000UL
#endif
// EEPROM
#define ADDRESS 0
#define NB_COUNTERS 4
#define MAGIC 0xA5
#define MAX_EEPROM_MEM (uint16_t)1024
// SRAM BUFFER
#define BUFFER_SIZE 255
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

void	ft_bzero(void *s, uint32_t n){
	uint32_t    i;

	i = -1;
	while (++i < n)
	{	
		*((unsigned char *)s) = 0;
		++s;
	}
}

/*
** -----------------------------------------------------------------------------
** EEPROM functions
** -----------------------------------------------------------------------------
*/
bool    is_id_free(uint16_t id){
    /*
    ** Returns :
    ** - 1 : if the memory address pointed by id is free
    ** - 0 : if the memory address pointed by id is reserved
    */

    uint16_t    size = 0;
    uint8_t     size_h = 0;
    uint8_t     size_l = 0;
    uint8_t     maybe_magic;

    // Search for previous magic number
    for (int i = id; i >= 0; i--) {
        maybe_magic = eeprom_read_byte((const uint8_t *)i);
        if (maybe_magic == MAGIC) {
            // Rebuild size
            size_h = eeprom_read_byte((const uint8_t *)i + 1);
            size_l = eeprom_read_byte((const uint8_t *)i + 2);
            size = size_h;
            size <<= 8;
            size |= size_l;
            // Compare size with current id
            if ((id - i) <= size + 3) {
                return (0);
            }
            else { return (1); }
        }
    }
    // No previous magic number : it's free !
    return (1);
}

bool    can_be_reserved(uint16_t id){

    // Limits
    if (id > MAX_EEPROM_MEM || id < 3) { return (0);}

    if (is_id_free(id - 1) && is_id_free(id - 2) && is_id_free(id - 3)){
        return (1);
    }
    return (0);
}

bool    is_free_space_large_enought(uint16_t id, size_t size){
    /*
    ** Returns :
    ** - 1 : if there is enought free space in eeprom to store size bytes from
    **       free space id
    ** - 0 : if there is somme reserved space between id and id + size
    */

    uint8_t maybe_magic;

    // Limits
    if (id + 3 + size > MAX_EEPROM_MEM) { return (0);}

    // Test for freeness
    for (uint16_t i = id; i < id + 3 + size; i++) {
        maybe_magic = eeprom_read_byte((const uint8_t *)i);
        if (maybe_magic == MAGIC) {
            return (0);
        }
    }
    // No magic number on the way : it's free !
    return (1);
}

bool    is_my_read_in_reserved_space(uint16_t id, size_t expected_size){
    /*
    ** Returns:
    ** - 1 : if I can read from id address to id + size with only reserved space
    ** - 0 : if I have a risk to read magic num / size or free space
    */

    uint16_t    size = 0;
    uint8_t     size_h = 0;
    uint8_t     size_l = 0;
    uint8_t     maybe_magic;

    // Limits
    if (id > MAX_EEPROM_MEM || id < 3 || id + size > MAX_EEPROM_MEM){ 
        return (0);
    }
    // Search for previous magic number
    for (int i = id; i >= 0; i--) {
        maybe_magic = eeprom_read_byte((const uint8_t *)i);
        if (maybe_magic == MAGIC) {
            // Rebuild size
            size_h = eeprom_read_byte((const uint8_t *)i + 1);
            size_l = eeprom_read_byte((const uint8_t *)i + 2);
            size = size_h;
            size <<= 8;
            size |= size_l;
            // Compare size with current id
            if (id + expected_size <= i + 3 + size) {
                return (1);
            }
            else { return (0); }
        }
    }
    // No previous magic number : it's free !
    return (1);

}

bool    eepromalloc_read(uint16_t id, void *buffer, size_t length){
    /*
    ** read data IF they have been written by me (check with my magic number)
    ** 
    ** - id : the memory offset (starting from 0)
    ** - buffer : the destination of the readed data
    ** - length : the length of the data to read
    */

    // Limits
    if (id > MAX_EEPROM_MEM || id < 3) { return (0);}

    // Check that the read is on an alloc'ed block and that the size fits
    if (!is_id_free(id)){
        // Checks that the size fits
        if (is_my_read_in_reserved_space(id, length)){
            // The read is safe
            eeprom_read_block((void *)buffer, (const void *)id, length);
            return (1);
        }
    }
    // The read is unsafe (free memory or size overflow)
    return (0);
}

bool    eepromalloc_write(uint16_t id, void *buffer, uint16_t length){
    /*
    ** write data with a magic number to identify them as written by me (or do
    ** not re-write them if it is already written by me)
    ** 
    ** - id : the memory where to write (starting from 0 to 1024)
    ** - buffer : the data to write
    ** - length : the length of the data to write
    */
    
    // If I cannot add magic for the address OR if I cannot write all the buffer
    if (id + length > MAX_EEPROM_MEM){
        return (0);
    }
    // Checks if required space is free
    if (can_be_reserved(id)){
        if (is_free_space_large_enought(id - 3, length)) {
            eeprom_write_byte((uint8_t *)(id - 3), MAGIC);
            eeprom_write_byte((uint8_t *)(id - 2), \
                              (uint8_t)((length & 0xFF00) >> 8));
            eeprom_write_byte((uint8_t *)(id - 1), \
                              (uint8_t)(length & 0x00FF));
            eeprom_write_block((const void *)buffer, (void *)(id), length);
            return (1);
        }
    }
    return (0);
}

bool    eepromalloc_free(uint16_t id){
    /*
    **  Free the space if id points to a magic number (replace it by 0)
    */

    // Limits
    if (id > MAX_EEPROM_MEM || id < 3) { return (0);}
    if (eeprom_read_byte((const uint8_t *)(id - 3)) == MAGIC) {
        eeprom_write_byte((uint8_t *)id - 3, (uint8_t)0);
        return (1);
    }
    return (0);
}

/*
** -----------------------------------------------------------------------------
** MAIN program
** -----------------------------------------------------------------------------
*/
int main(void){

    bool    res;
    char    buf[BUFFER_SIZE] = {0};

    // Initialize UART
    uart_init(UART_BAUDRATE, 0);

    // -------------------------------------------------------------------------
    // TEST 1 : First write
    // -------------------------------------------------------------------------
    // Free address
    eepromalloc_free(3);
    // Write on address
    uart_printstr("\n\rTEST 1\n\r");
    // write in eeprom
    res = eepromalloc_write(3, "Ceci est un test", 16);
    if (res == 0){
        uart_printstr("Error\n\r");
        return (1);
    }

    // Read from eeprom
    res = eepromalloc_read(3, (void *)buf, 16);
    if (res == 0){
        uart_printstr("Error\n\r");
        return (1);
    }
    uart_printstr(buf);
    uart_printstr("\n\r");

    // -------------------------------------------------------------------------
    // TEST 2 : Write on reserved space
    // -------------------------------------------------------------------------
    uart_printstr("\n\rTEST 2\n\r");
    // write in eeprom
    res = eepromalloc_write(3, "Ceci est un test", 16);
    if (res == 0){
        uart_printstr("Error with write params\n\r");
    }

    // -------------------------------------------------------------------------
    // TEST 3 : read too much
    // -------------------------------------------------------------------------
    uart_printstr("\n\rTEST 3\n\r");
    // Read from eeprom
    res = eepromalloc_read(3, (void *)buf, 17);
    if (res == 0){
        uart_printstr("Error with read params\n\r");
    }

    // -------------------------------------------------------------------------
    // TEST 4 : read in param zone
    // -------------------------------------------------------------------------
    uart_printstr("\n\rTEST 4\n\r");
    // Read from eeprom
    res = eepromalloc_read(2, (void *)buf, 8);
    if (res == 0){
        uart_printstr("Error with read params\n\r");
    }

    // -------------------------------------------------------------------------
    // TEST 5 : write on reserved space
    // -------------------------------------------------------------------------
    uart_printstr("\n\rTEST 5\n\r");
    // write in eeprom
    res = eepromalloc_write(15, "Ceci est encore un test", 23);
    if (res == 0){
        uart_printstr("Error with write params\n\r");
    }

    // -------------------------------------------------------------------------
    // TEST 6 : Not enought space to write
    // -------------------------------------------------------------------------
    uart_printstr("\n\rTEST 6\n\r");
    // write in eeprom
    res = eepromalloc_write(1020, "Ceci est encore un test", 23);
    if (res == 0){
        uart_printstr("Error with write params\n\r");
    }

    // -------------------------------------------------------------------------
    // TEST 7 : write ok
    // -------------------------------------------------------------------------
    uart_printstr("\n\rTEST 7\n\r");
    // write in eeprom
    eepromalloc_free(1000);
    ft_bzero(buf, BUFFER_SIZE);
    res = eepromalloc_write(1000, "Ceci est encore un test", 23);
    if (res == 0){
        uart_printstr("Error with write params\n\r");
    }

    // Partial read from eeprom
    res = eepromalloc_read(1009, (void *)buf, 14);
    if (res == 0){
        uart_printstr("Error\n\r");
        return (1);
    }
    uart_printstr(buf);
    uart_printstr("\n\r");

    // Loop
    while (1) {
    }

    return (0);
}
