#ifndef TIMER_H
# define TIMER_H

/*
** -----------------------------------------------------------------------------
** Params
** -----------------------------------------------------------------------------
*/
# include <avr/io.h>
# include "utils.h"

/*
** -----------------------------------------------------------------------------
** General
** -----------------------------------------------------------------------------
*/
void    timer_0_conf(uint8_t mode, uint16_t prescale, uint8_t comp_a,\
                     uint8_t frequency);
void    timer_1_conf(uint8_t mode, uint16_t prescale, uint16_t comp_a,\
                     float frequency);
void    timer_2_conf(uint8_t mode, uint16_t prescale, uint8_t comp_a,\
                     uint8_t frequency);

void    stop_timer_0(void);
void    stop_timer_1(void);
void    stop_timer_2(void);

#endif