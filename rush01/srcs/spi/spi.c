#include "spi.h"

/*
** -----------------------------------------------------------------------------
** SPI Functions
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
    (void)trash;
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
