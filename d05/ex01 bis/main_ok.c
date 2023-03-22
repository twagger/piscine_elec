#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/twi.h>
#include <util/delay.h>

/*
•use the EEPROM to save and restore the state of 4 counters.
•use the EEPROM to save and restore the current selected counter.
•use the button SW2 to increase the count.
•use the button SW3 to select a counter.
•use the LEDs on the board to show the current state of the selected counter.
*/

# ifndef F_CPU
# define F_CPU 16000000UL
# endif

# ifndef TWI_FREQ
# define TWI_FREQ 100000L 
# endif

# ifndef UART_BAUDRATE
# define UART_BAUDRATE 115200
# endif

# ifndef MYUBRR
# define MYUBRR ((((F_CPU)/(8 * UART_BAUDRATE)) - 1)/2)
# endif

#ifndef DEBUG
# define DEBUG 0
#endif

int ERRNO;

void	uart_init(unsigned int ubrr) // for Fosc 16MHz and Baud 115.2Kbps ubrr = 8
{
	// ubrrn p165
	/*Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1<<TXEN0)|(1<<RXEN0); // Enable transmitter & receiver
	UCSR0C = (0<<USBS0)|(3<<UCSZ00); // frame format 8N1 aka 8 data and 1 stop bit
}

void	uart_tx(char c)
{
	/* Wait for empty transmit buffer */
	while (!(UCSR0A & (1<<UDRE0)))
	;
	/* Put data into buffer, sends the data */
	UDR0 = (unsigned char)c;
}

void uart_printstr(const char* str)
{
	while (str && *str)
		uart_tx(*str++);
}

char uart_rx(void)
{
	/* Wait for data to be received */
	while (!(UCSR0A & (1<<RXC0)))
	;
	/* Get and return received data from buffer */
	return UDR0;
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

void ERROR(const char *IDE)
{
	uart_printstr("ERROR @");
	uart_printstr(IDE);
	uart_printstr("\n\r");
	ERRNO = 1;
}

void SUCCESS(const char *IDS)
{
	_delay_ms(1);
	ERRNO = 0;
	if (!DEBUG || !IDS)
		return ;
	uart_printstr("SUCCESS @");
	uart_printstr(IDS);
	uart_printstr("\n\r");
}

void i2c_init(void)
{
	// TWBR = ((CPU Clock frequency / SCL frequency) – 16) / (2 * 4 ^ TWPS)
	TWBR = (((F_CPU / TWI_FREQ) - 16) / 2);
	TWCR = (1<<TWEN); // Enable I2C (and set own slave address)
}

void i2c_wait(void)
{
	static int a;
	char b[] = "/|\\-";
	while (!(TWCR & (1<<TWINT)))
	{
		if (DEBUG)
			uart_tx(b[a++ % (sizeof(b) - 1)]), uart_tx('\b'), _delay_ms(30);
	}
}

int i2c_start(int SLADR, _Bool R)
{
	// TWSTA: TWI START Condition Bit -- Send START condition.
	// Clear when START condition sent (TWINT = 1).
	// TWEN: TWI ENable Bit -- Activate TWI peripheral subsystem.
	// TWI takes control over the I/O pins connected to the SCL and SDA pins.
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN); // Send START condition
	i2c_wait(); // Wait for TWINT flag set. This indicates that the START condition has been transmitted
	if ((TWSR & 0xF8) != TW_START && (TWSR & 0xF8) != TW_REP_START) // START is 0x08, 0xF8 is 'no relevant state info avail, TWINT=0'
		return (ERROR("Start"), 1);
	if (!SLADR)
		return (SUCCESS("Just start"), 0);
	TWDR = (SLADR << 1) + R; // 7 first bytes are sensor address, last one is 0 for Write or 1 for Read
	TWCR = (1<<TWINT)|(1<<TWEN); // start transmission of address
	i2c_wait(); // Wait for TWINT flag set. This indicates that the SLA+W has been transmitted, and ACK/NACK has been received.
	if ((TWSR & 0xF8) != (R ? TW_MR_SLA_ACK : TW_MT_SLA_ACK)) // Check value of TWI status register. Mask prescaler bits. If status different from MT_SLA_ACK go to ERROR
		return (ERROR(R ? "R start no ack" : "W start no ack"), 1);
	return (SUCCESS("start"), 0);
}

void i2c_stop(void)
{
	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN); // Send STOP condition
	// SUCCESS("Stop?");
}

void i2c_write(unsigned char data)
{
	if (ERRNO)
		return ;
	TWDR = data; // Load DATA into TWDR register. 
	TWCR = (1<<TWINT) | (1<<TWEN); // clear TWINT bit in TWCR to start transmission of data
	i2c_wait(); // Wait for TWINT flag set. This indicates that the DATA has been transmitted, and ACK/NACK has been received
	if ((TWSR & 0xF8) != TW_MT_DATA_ACK) // Check value of TWI status register. Mask prescaler bits. If status different from MT_DATA_ACK go to ERROR
		ERROR(__func__);
	else
		SUCCESS(__func__);
}

char i2c_read(_Bool ACK_NEXT)
{
	if (ERRNO)
		return '\0';
	TWCR = (1<<TWINT) | (1<<TWEN) | (ACK_NEXT<<TWEA);
	i2c_wait(); // Wait for TWINT flag set. This indicates that the DATA has been received, and ACK/NACK has been transmitted ?
	if ((TWSR & 0xF8) != TW_MR_DATA_ACK && (TWSR & 0xF8) != TW_MR_DATA_NACK) // Check value of TWI status register. Mask prescaler bits. If status different from MT_DATA_ACK go to ERROR
		ERROR(__func__);
	else
		SUCCESS(__func__);
	return TWDR;
}

int pressing_sw3(void)
{
	i2c_start(0b0100000, 0);	// write to expander
	i2c_write(6);				// command 6 for port 0 config
	i2c_write(1);				// set port0 to 1 aka input
	i2c_stop();
	i2c_start(0b0100000, 0);	// write to expander
	i2c_write(0);				// port 0 input
	i2c_start(0b0100000, 1);	// read
	int sw3 = i2c_read(0);		// value of port 0
	uart_byte_printer(sw3);
	i2c_stop();
	return (!(sw3 & 1));		// mask for value of port 0's pin 0 for SW3's state
}

void light_binary(uint8_t val)
{
	// DDRB = (1<<DD0)|(1<<DD1)|(1<<DD2)|(1<<DD4);
	PORTB =	 ((val / 1 % 2)<<PORT0)
			|((val / 2 % 2)<<PORT1)
			|((val / 4 % 2)<<PORT2)
			|((val / 8 % 2)<<PORT4);
}

int main(void)
{
	uart_init(MYUBRR);
	i2c_init();
	DDRB = (1<<DD0)|(1<<DD1)|(1<<DD2)|(1<<DD4); // D1-4 are on PB0 1 2 and 4 !
	DDRD = ~(1<<DD2)&~(1<<DD4);	// SW1 is on PD2, SW2 on PD4
	PORTB = (0<<PORT0)|(0<<PORT1)|(0<<PORT2)|(0<<PORT4); // useless but eh
	int	rel = 0;
	uint8_t *addr = (uint8_t *)69;
	uint8_t	counter = eeprom_read_byte(addr); // get counter from addr
	uint8_t	val;
	while (1)
	{
		val = eeprom_read_byte(addr + counter); // get val from current counter
		if (!(PIND & (1<<DD2))) // check whether SW1 is pressed down or not
		{
			eeprom_write_byte(addr + 1, 0); // reset all counters
			eeprom_write_byte(addr + 2, 0);
			eeprom_write_byte(addr + 3, 0);
			eeprom_write_byte(addr + 4, 0);
			counter = 1;
			eeprom_write_byte(addr, counter); // select counter 1
			val = 0;
			light_binary(val);
		}
		else if (!(PIND & (1<<DD4))) // check whether SW2 is pressed down or not
		{
			if (!rel)
				val++;
			val %= 16;
			eeprom_write_byte(addr + counter, val); // save current value of val in current counter
			light_binary(val);
			_delay_ms(100);
			rel = 1;
		}
		else if (pressing_sw3()) // check whether SW2 is pressed down or not
		{
			if (!rel)
				counter = counter == 4 ? 1 : counter + 1;
			eeprom_write_byte(addr, counter); // save new value of counter
			val = eeprom_read_byte(addr + counter); // get new counter's val
			light_binary(val);
			_delay_ms(100);
			rel = 1;
		}
		else
			rel = 0;
	}
}
