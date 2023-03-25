#include "timer.h"

/*
** -----------------------------------------------------------------------------
** Timer Functions
** -----------------------------------------------------------------------------
*/

void    timer_0_conf(uint8_t mode, uint16_t prescale, uint8_t comp_a, \
                     uint8_t frequency) {
    /*
    ** This function configures the timer 0.
    */

    // 1. Waveform Generation Mode
    // Clear registers
    TCCR0A &= 0xFC;
    TCCR0B &= 0xF7;
    // Set registers
    TCCR0A |= (mode & 0x03);
    TCCR0B |= (((mode >> 2) & 0x01) << 3);
    // 3. Value to compare the timer with
    OCR0A = comp_a;
    // 3. Interrupts when compare match
    TIMSK0 |= (1 << OCIE0A);
    // 4. Clock prescale factor
    TCCR0B |= ((uint32_t)(my_log2(prescale) / 2) << CS00);
}

void    timer_1_conf(uint8_t mode, uint16_t prescale, uint16_t comp_a, \
                     uint8_t frequency) {
    /*
    ** This function configures the timer 1
    */

    // 1. Action on OC1A (PB1) on compare match. Here : nothing
    TCCR1A &= ~((1 << COM1A0) | (1 << COM1A1));
    // 2. Waveform Generation Mode
    // Clear registers
    TCCR1A &= 0xFC;
    TCCR1B &= 0xF7;
    // Set registers
    TCCR1A |= (mode & 0x03);
    TCCR1B |= (((mode >> 2) & 0x01) << 3);
    // 3. Value to compare the timer with
    OCR1A = (F_CPU / (2 * 256 * frequency) - 1);
    // 4. Enable and configure timer 1 interrupt
    TIMSK1 |= (1 << OCIE1A);
    // 4. Clock prescale factor + launch the timer
    TCCR1B |= ((uint32_t)(my_log2(prescale) / 2) << CS10);
}

void    timer_2_conf(uint8_t mode, uint16_t prescale, uint8_t comp_a, \
                     uint8_t frequency) {
    /*
    ** This function configures the timer 2.
    */

    // 1. Waveform Generation Mode
    // Clear registers
    TCCR2A &= 0xFC;
    TCCR2B &= 0xF7;
    // Set registers
    TCCR2A |= (mode & 0x03);
    TCCR2B |= (((mode >> 2) & 0x01) << 3);
    // 3. Value to compare the timer with
    OCR2A = comp_a;
    // 3. Interrupts when compare match
    TIMSK2 |= (1 << OCIE2A);
    // 4. Clock prescale factor
    TCCR2B |= ((uint32_t)(my_log2(prescale) / 2) << CS20);
}

