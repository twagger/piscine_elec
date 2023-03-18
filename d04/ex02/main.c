// AVR
#include <avr/io.h>
#include <util/twi.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#ifndef F_CPU
# define F_CPU 16000000UL
#endif
// I2C
#define SENSOR_W 0x70
#define SENSOR_R 0x71
#define MEASUREMENT_PACKETS 7
// UART
#define UART_BAUDRATE 115200
#define __INTR_ATTRS used, externally_visible
#define ISR(vector, ...)            \
        void vector (void) __attribute__ ((signal,__INTR_ATTRS)) __VA_ARGS__; \
        void vector (void)


// Enums
enum    e2i_options{
    OPT_DATA = 0,
    OPT_START,
    OPT_STOP,
    OPT_RESTART,
};

// Globals
static uint8_t          i2c_error = 0;
static const char       *status_mess[] = {\
                                "A START condition has been transmitted", \
                                "SLA+W has been transmitted; ACK received", \
                                "SLA+R has been transmitted; ACK received", \
                                "Data byte has been transmitted; ACK received"};
static const uint8_t    status_codes[] = {TW_START, TW_MT_SLA_ACK,
                                          TW_MR_SLA_ACK, TW_MT_DATA_ACK};
static const uint8_t    debug = 0;
static uint32_t         temps[3] = {0, 0, 0};
static uint32_t         hums[3] = {0, 0, 0};
static uint8_t          meaner = 0;

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

/*
** -----------------------------------------------------------------------------
** Helper Functions
** -----------------------------------------------------------------------------
*/
int		ft_strlen(char *str){
	int i;

	i = 0;
	while (str[i] != '\0'){
		i++;
	}
	return (i);
}

void	ft_uartnbr_base(uint8_t nbr, char *base){
	uint8_t     i = 0;
	uint8_t     ibase = ft_strlen(base);;
	char	    nb_temp[3] = {'0', '0', '0'};

	while (nbr > 0) {
        nb_temp[i] = base[nbr % ibase];
        nbr = nbr / ibase;
        i++;
    }
    uart_tx(nb_temp[1]);
    uart_tx(nb_temp[0]);
}

void    ft_byte_printer(unsigned char c) {
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
        TWCR |= (1 << TWSTA) | (1 << TWINT) | (1 << TWEN);
        wait_for_transmission();
        if (check_for_status(expected) == 1) {
            return;
        }
    }
    else if (option == OPT_STOP) {
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    }
    else if (option == OPT_RESTART) {
        // To be developped
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

    // Enable ACK bit
    // Maybe to do 
    // TWCR |= (1 << TWEA);

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

void    i2c_write(unsigned char data){
    /*
    ** Write a byte to I2C bus
    */

    send_to_i2c(OPT_DATA, data, TW_MT_DATA_ACK);
    if (i2c_error) { return; }
}

int i2c_read(void){
    /*
    ** Display the content of TWDR after the sensor measurement
    */

    wait_for_transmission();
    return TWDR;
}

void    print_hex_value(char c){
    /*
    ** Display 1 bytes in Hexa
    */

    ft_uartnbr_base(c, "0123456789ABCDEF");
}

void    i2c_stop(void){
    /*
    ** Stops the I2C communication between the MCU and the sensor. The bus is no
    ** longer busy.
    */

    send_to_i2c(OPT_STOP, 0, 0);
}

/*
** -----------------------------------------------------------------------------
** SENSOR functions
** -----------------------------------------------------------------------------
*/
void    trigger_measurement(){
    /*
    ** Send the address of the command to trigger and the proper value
    */

    // Trigger measurement addr
    i2c_write(0xAC);
    if (i2c_error) { return; }

    // Trigger measurement data
    i2c_write(0x33);
    if (i2c_error) { return; }
    i2c_write(0x00);
    if (i2c_error) { return; }
    
    // Stop i2c
    i2c_stop();
}

void    read_measurement(){
    /*
    ** Read the measurement from the sensor
    */
    unsigned char   tmp;
    uint8_t         index;

    // Start in read mode
    i2c_start(SENSOR_R, TW_MR_SLA_ACK);
    if (i2c_error) { return; }

    // Read 7 packets
    for (uint8_t i = 0; i < MEASUREMENT_PACKETS; i++) {
        // Configure the ACK
        TWCR |= (1 << TWEA);
        // read and store into an array
        if (i == MEASUREMENT_PACKETS - 1){ // Last data byte
            tmp = i2c_read();
            // send nack and check
            TWCR = (1 << TWINT) | (1 << TWEN);
            wait_for_transmission();
            if (check_for_status(TW_MR_DATA_NACK) == 1) {
                return;
            }
        } else {
            tmp = i2c_read();
            index = meaner % 3;
            // save tmp in humidity / temperature array
            if (i == 1) { hums[index] = tmp; }
            if (i == 2) { hums[index] = (hums[index] << 8) | tmp; }
            if (i == 3) { hums[index] = (hums[index] << 4) | (tmp >> 4);
                          temps[index] = tmp & 0xF; }
            if (i == 4) { temps[index] = (temps[index] << 8) | tmp; }
            if (i == 5) { temps[index] = (temps[index] << 8) | tmp; }
            // send ack and check
            TWCR = (1 << TWEA) | (1 << TWINT) | (1 << TWEN);
            wait_for_transmission();
            if (check_for_status(TW_MR_DATA_ACK) == 1) {
                return;
            }
        }
    }

    // Stop i2c
    i2c_stop();
}

/*
** -----------------------------------------------------------------------------
** MAIN program
** -----------------------------------------------------------------------------
*/
int main(void){

    float   temp = 0.0;
    float   hum = 0.0;
    char    a_temp[25];
    char    a_hum[5];

    // Initialize UART to transfer sensor and I2C infos
    uart_init(UART_BAUDRATE, 0);

    // Init I2C
    i2c_init();

    // debug
    DDRB |= (1 << PB0);

    // Wait 40ms before asking for a measurement
    _delay_ms(40);

    while (1) {

        // Start transmission
        i2c_start(SENSOR_W, TW_MT_SLA_ACK);
        if (i2c_error) { return (1); }

        // Ask for data from the sensor    
        trigger_measurement();
        if (i2c_error) { return (1); }

        // Read the result and display hex value
        _delay_ms(80);
        read_measurement();
        if (i2c_error) { return (1); }

        // Display temp
        if (index == 0 && meaner != 0){ //error
            // mean the temp and humidity
            temp = (temps[0] + temps[1] + temps[2]) / 3;
            uart_printstr(dtostrf(temps[0], 4, 0, a_temp));
            temp = (temp / pow(2,20)) * 200 - 50;
            hum = (hums[0] + hums[1] + hums[2]) / 3;
            hum = (hum / pow(2,20)) * 100;
            // display temp and humidity
            uart_printstr("Temperature: ");
            uart_printstr(dtostrf(temp, 4, 0, a_temp));
            uart_printstr(".C, Humidity: ");
            uart_printstr(dtostrf(hum, 4, 1, a_hum));
            uart_printstr("%\n\r");
        }
        ++meaner;

        // Delay to have time to read the temperature on the screen
        _delay_ms(3000);
    }

    return (0);
}
