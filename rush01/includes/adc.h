#ifndef ADC_H
# define ADC_H

/*
** -----------------------------------------------------------------------------
** Params
** -----------------------------------------------------------------------------
*/
# include <avr/io.h>
// Devices
# define RV1 0
# define LDR 1
# define NTC 2

/*
** -----------------------------------------------------------------------------
** Functions
** -----------------------------------------------------------------------------
*/
void    adc_init(uint8_t precision);
void    start_conversion(void);
void    select_device(uint8_t device);

#endif