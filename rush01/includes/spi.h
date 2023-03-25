#ifndef SPI_H
# define SPI_H

/*
** -----------------------------------------------------------------------------
** Params
** -----------------------------------------------------------------------------
*/
# include <avr/io.h>
// SPI – Serial Peripheral Interface
# define MOSI PB3
# define MISO PB4
# define SCK PB5
# define SS PB2
// Colors
# define RED 0
# define GREEN 1
# define BLUE 2
// SPI LEDS
# define D6 0
# define D7 1
# define D8 2
# define BRIGHTNESS 0x01

/*
** -----------------------------------------------------------------------------
** Functions
** -----------------------------------------------------------------------------
*/
void    spi_wait(void);
void    spi_master_init(void);
void    spi_write(uint8_t byte);
void    spi_apa102_start_frame(void);
void    spi_apa102_end_frame(void);
void    spi_led(uint32_t color, uint8_t brightness);

#endif