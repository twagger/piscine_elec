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
// Program specific
#define BACKSPACE 127
#define NEWLINE '\r'
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
volatile static uint8_t         command_is_entered = 0;
volatile static unsigned char   buffer[BUFFER_SIZE] = {0};
volatile static char            *keywords[3] = {"READ", "WRITE", "FORGET"};
volatile const static uint8_t   nb_keywords = sizeof(keywords) / sizeof(keywords[0]);

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

int	ft_strlen(char *s){
	int	i;

	if (!s)
		return (0);
	i = 0;
	while (*(s + i))
		++i;
    return (i);
}

int ft_strchr(unsigned char *s, int c){
    uint16_t i = 0;
    if (!s)
		return (-1);
	while (s[i])
	{
		if (s[i] == (char)c)
			return (i);
		++i;
	}
	if (c == 0)
		return (0);
	return (-1);
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

    uint16_t    size_value = 0;
    uint8_t     size_key = 0;
    uint8_t     size_value_h = 0;
    uint8_t     size_value_l = 0;
    uint8_t     maybe_magic;

    // Search for previous magic number
    for (int i = id; i >= 0; i--) {
        maybe_magic = eeprom_read_byte((const uint8_t *)i);
        if (maybe_magic == MAGIC) {
            // Get size key
            size_key = eeprom_read_byte((const uint8_t *)i + 1);
            // Rebuild size value
            size_value_h = eeprom_read_byte((const uint8_t *)i + 2);
            size_value_l = eeprom_read_byte((const uint8_t *)i + 3);
            size_value = size_value_h;
            size_value <<= 8;
            size_value |= size_value_l;
            // Compare size with current id
            if ((id - i) <= size_key + size_value + 4) {
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
    if (id > MAX_EEPROM_MEM || id < 4) { return (0);}

    if (is_id_free(id - 1) && is_id_free(id - 2) && is_id_free(id - 3) \
        && is_id_free(id - 4)){
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
    if (id + 4 + size > MAX_EEPROM_MEM) { return (0);}

    // Test for freeness
    for (uint16_t i = id; i < id + 4 + size; i++) {
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

bool    eepromalloc_write(uint16_t id, void *key, void *value, \
                          uint8_t length_key, uint16_t length_value){
    /*
    ** write data with a magic number to identify it and lengths (key / value)
    */
    
    // If I cannot add magic for the address OR if I cannot write all the buffer
    if (id + length_key + length_value > MAX_EEPROM_MEM){
        return (0);
    }
    // Checks if required space is free
    if (can_be_reserved(id)){
        if (is_free_space_large_enought(id - 4, length_key + length_value)) {
            eeprom_write_byte((uint8_t *)(id - 4), MAGIC);
            eeprom_write_byte((uint8_t *)(id - 4), length_key);
            eeprom_write_byte((uint8_t *)(id - 2), \
                              (uint8_t)((length_value & 0xFF00) >> 8));
            eeprom_write_byte((uint8_t *)(id - 1), \
                              (uint8_t)(length_value & 0x00FF));
            eeprom_write_block((const void *)key, (void *)(id), length_key);
            eeprom_write_block((const void *)value, (void *)(id + length_key), \
                                length_value);
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
** Interrupts
** -----------------------------------------------------------------------------
*/
ISR(USART_RX_vect){
    unsigned char   c;
    static uint8_t  rx_write_pos = 0;

    // Read char from buffer    
    c = UDR0;

    // Save char into the proper buffer and display it
    if (c == BACKSPACE) {
        if (rx_write_pos != 0) {
            --rx_write_pos;
            buffer[rx_write_pos] = '\0';
            uart_printstr("\b \b");
        }
    }
    else if (c == NEWLINE) {
        command_is_entered = 1;
        rx_write_pos = 0;
        uart_printstr("\r\n");
    }
    else {
        buffer[rx_write_pos] = c;
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
** Program helpers
** -----------------------------------------------------------------------------
*/
void    prompt(){
    uart_printstr("> ");
    while (command_is_entered == 0)
        ;
    command_is_entered = 0;
}

void    display_error(){
    uart_printstr("Bad command format (READ / WRITE / FORGET + <str>)\n\n\r");
}

uint8_t is_command_valid(void){
    /*
    ** Checks if the command is valid
    */
    uint8_t kw_len;

    for (uint8_t i = 0; i < nb_keywords; i++){
        kw_len = ft_strlen((char *)keywords[i]);
        // Keyword
        if (ft_strncmp((const char *)buffer, (const char *)keywords[i], kw_len) == 0) {
            // Space
            if (buffer[kw_len] && buffer[kw_len] == ' ') {
                // Double quote
                if (buffer[kw_len + 1] && buffer[kw_len + 1] == '"') {
                    // Final Double quote
                    if (ft_strchr((unsigned char *)&(buffer[kw_len + 2]), '"') != - 1) {
                        return (1);
                    }
                }
            }
        }
    }
    
    return (0);
}

uint8_t read(char *key) {
    // Search KEY in reserved EEPROM
    // 1. Search for magic keyword
    // 2. read the lenght of the key
    // 3. compare the key with the param key
    // 4. if this is the right key, print the value
    // 5. else jump to the next
}

uint8_t write(char *key, char *value){
    // 1. Search if key already exists (error if it already exists, you need to 
    // forget it firtst)
    // 2. Search from 0 to the next free space with m (1) + lk (1) + lv (1) + key + value
    // 3. if space : write and return 1
    // 4. if no space : no write and return 0
}

uint8_t forget(char *key){
    // 1. Search if key exists
    // 2. if it don't exists : return 0
    // 3. if it exists : remove magic number and return 1
}


uint8_t execute_command(){

    // READ
    if (ft_strncmp((const char *)buffer, "READ", 4) == 0) {
        key = buffer + 5;
        return read(key);
    }
    // WRITE
    else if (ft_strncmp((const char *)buffer, "WRITE", 5) == 0) {
        key = 
        return write(buffer + 6);
    }
    // or FORGET
    else if (ft_strncmp((const char *)buffer, "FORGET", 6) == 0) {
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

    // Loop
    while (1) {
        // Prompt
        prompt();
        if (is_command_valid()){
            uart_printstr("Command valid ! \n\r");
            if (execute_command() == 0) {
                uart_printstr("Error in command execution\n\r");
            }
        } else {
            display_error();
        }
    }

    return (0);
}
