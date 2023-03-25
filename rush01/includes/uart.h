#ifndef UART_H
# define UART_H

/*
** -----------------------------------------------------------------------------
** Params
** -----------------------------------------------------------------------------
*/
# include <avr/io.h>
# define UART_BAUDRATE 115200
# define __INTR_ATTRS used, externally_visible
# define ISR(vector, ...)            \
        void vector (void) __attribute__ ((signal,__INTR_ATTRS)) __VA_ARGS__; \
        void vector (void)

/*
** -----------------------------------------------------------------------------
** Functions
** -----------------------------------------------------------------------------
*/
void            uart_init(uint32_t baud, uint8_t double_speed);
void            uart_tx(char c);
unsigned char   uart_rx(void);
void            uart_printstr(const char *str);
void            uart_byte_printer(unsigned char c);

#endif