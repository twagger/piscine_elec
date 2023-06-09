#ifndef I2C_H
# define I2C_H

/*
** -----------------------------------------------------------------------------
** Params
** -----------------------------------------------------------------------------
*/
# include <avr/io.h>
# include <util/twi.h>
# include <util/delay.h>
# include "uart.h"
# define EXPANDER_W 0b01000000
# define EXPANDER_R 0b01000001
# define OPT_DATA 0
# define OPT_START 1
# define OPT_STOP 2
# define D11 1
# define D10 2
# define D09 3
# define DASH 0x40

/*
** -----------------------------------------------------------------------------
** Globals
** -----------------------------------------------------------------------------
*/
// General
extern uint8_t          i2c_error;
extern const uint8_t    debug;
extern const char       *status_mess[];
extern const uint8_t    status_codes[];
// 7 segments
extern const uint8_t    brightness[]; // ratio on / off on the 7seg
extern const uint8_t    numbers[];

/*
** -----------------------------------------------------------------------------
** General
** -----------------------------------------------------------------------------
*/
void    wait_for_transmission(void);
int     check_for_status(uint8_t expected);
void    send_to_i2c(uint8_t option, unsigned char data, uint8_t expected);
void    i2c_init(void);
void    i2c_start(uint8_t data, uint8_t expected);
void    i2c_repeat_start(uint8_t data, uint8_t expected);
void    i2c_write(unsigned char data);
uint8_t i2c_read(void);
void    i2c_stop(void);
void    send_nack(void);
void    handle_i2c_error(void);

/*
** -----------------------------------------------------------------------------
** Expander
** -----------------------------------------------------------------------------
*/
void    init_i2e_expander(uint8_t port0, uint8_t port1);

/*
** -----------------------------------------------------------------------------
** 7 Segments
** -----------------------------------------------------------------------------
*/
void    clear_all_digits(void);
void    display_one_digit(uint8_t number, uint8_t position, uint8_t dot);
void    display_one_sign(uint8_t sign, uint8_t position, uint8_t dot);

/*
** -----------------------------------------------------------------------------
** I2C Leds
** -----------------------------------------------------------------------------
*/
void    toogle_i2c_led(uint8_t led);
void    turn_on_i2c_led(uint8_t led);
void    turn_off_i2c_led(uint8_t led);

#endif