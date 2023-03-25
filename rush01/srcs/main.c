// AVR
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#ifndef F_CPU
# define F_CPU 16000000UL
#endif
// Custom functions
#include "adc.h"
#include "i2c.h"
#include "uart.h"
#include "spi.h"
#include "timer.h"
#include "utils.h"
#include "switch.h"
#include "rgb.h"
// Switch
#define DEBOUNCING 250 // in ms

/*
** -----------------------------------------------------------------------------
** Prototypes
** -----------------------------------------------------------------------------
*/
void    mode0(void);
void    mode1(void);
void    mode2(void);
void    mode3(void);
void    mode4(void);
void    mode5(void);
void    mode6(void);
void    mode7(void);
void    mode8(void);
void    mode9(void);
void    mode10(void);

/*
** -----------------------------------------------------------------------------
** Globals
** -----------------------------------------------------------------------------
*/
// 7 segment
static volatile uint16_t    seg_value = 0;
static volatile uint16_t    seg_dot = 0;
static volatile uint8_t     sign[] = {0,0,0,0};
// Timer
volatile uint32_t           time_count = 0;
// Direct leds
static const uint8_t        leds[] = {PB0, PB1, PB2, PB4};
static const uint8_t        nb_leds = sizeof(leds) / sizeof(leds[0]);
// Direct switchs
static const uint8_t        switchs[] = {PD2, PD4};
static const uint8_t        nb_switchs = sizeof(switchs) / sizeof(switchs[0]);
// Modes
static uint8_t              mode = 0;
static void                 (*modes[])(void) = {mode0, mode1, mode2, mode3,
                                                mode4, mode5, mode6, mode7,
                                                mode8, mode9, mode10};
static const uint8_t        nb_mode = sizeof(modes) / sizeof(modes[0]);
// RGB
volatile static uint32_t    colors[] = {0, 0, 0, 0};
const static uint32_t       wheel[] = {0xFF0000, 0x00FF00, 0x0000FF};
static const uint8_t        nb_colors = sizeof(wheel) / sizeof(wheel[0]);
static volatile uint8_t     selected_color = 0;


/*
** -----------------------------------------------------------------------------
** Helpers
** -----------------------------------------------------------------------------
*/
void    my_wait(uint32_t ms){
    uint32_t    start = time_count;
    while (time_count - start < ms)
        ;
}

void    display_bin(void){
    /*
    ** Display the 4 LSB of a number using 4 leds of the board
    */

    for (uint8_t i = 0; i < nb_leds; i++) {
        if (mode & 1 << i){
            PORTB |= (1 << leds[i]);
        }
        else { PORTB &= ~(1 << leds[i]); }
    }
}

void    clear(void){
    // Clear 7seg
    clear_all_digits();
    seg_dot = 0;
    // Disable ADC
    disable_adc();
    // Disable SPI
    if (SPCR & (1 << SPE)) {
        display_colors_on_leds(0,0,0);
        disable_spi();
    }
    // Clear signs
    for (uint8_t i = 0; i < 4; i++) {
        sign[i] = 0;
    }
    // Stop timers
    stop_timer_0();
    stop_timer_1();
    stop_timer_2();
    // Set all color off
    PORTD &= ~((1 << DEL_RED) | (1 << DEL_GREEN) | (1 << DEL_BLUE));
}

/*
** -----------------------------------------------------------------------------
** Interrupts
** -----------------------------------------------------------------------------
*/
// 7 segment refresh
ISR(TIMER0_COMPA_vect){
    /*
    ** Timer to update digit and display it on 7 seg
    */
    if (sign[0] != 0){ display_one_sign(sign[0], 1, seg_dot); }
    else { display_one_digit(seg_value % 10, 1, seg_dot); }
    
    if (sign[1] != 0){ display_one_sign(sign[1], 2, seg_dot); }
    else { display_one_digit((seg_value / 10) % 10, 2, seg_dot); }

    if (sign[2] != 0){ display_one_sign(sign[2], 3, seg_dot); }
    else { display_one_digit((seg_value / 100) % 10, 3, seg_dot); }

    if (sign[3] != 0){ display_one_sign(sign[3], 4, seg_dot); }
    else { display_one_digit((seg_value / 1000) % 10, 4, seg_dot); }
}

ISR(TIMER1_COMPA_vect){
    /*
    ** Timer to display and rotate rgb leds every second
    */
    
    // Show colors
    display_colors_on_leds(wheel[selected_color], wheel[selected_color], \
                           wheel[selected_color]);
    display_rgb(wheel[selected_color]);

    // Move to next color
    selected_color = (selected_color + 1 >= nb_colors) ? 0 : selected_color + 1;
}


ISR(TIMER2_COMPA_vect){
    time_count++;
}


// SW1
ISR(INT0_vect){
    // Turn on D09 for test
    turn_off_i2c_led(D09);

    // Clear before changing mode
    clear();

    // Change mode
    mode = (mode + 1 >= nb_mode) ? 0 : mode + 1;
    display_bin();
    modes[mode]();

    // Debounce + clear int flag
    _delay_ms(DEBOUNCING);
    EIFR = (1 << INTF0); 
}

// SW2
ISR(PCINT2_vect){
    if ((PIND & (1 << PD4)) == 0) { // On switch press
        // Turn on D10for test
        turn_on_i2c_led(D10);

        // Clear before changing mode
        clear();

        // Change mode
        mode = (mode - 1 < 0) ? nb_mode - 1 : mode - 1;
        display_bin();
        modes[mode]();

        // Debounce + clear int flag
        _delay_ms(DEBOUNCING);
        EIFR = (1 << INTF0);
    } else { // On switch release
        turn_off_i2c_led(D10);
        _delay_ms(DEBOUNCING);
        EIFR = (1 << INTF0); // clear external interrupt flag 
    }
}

// ADC
ISR(ADC_vect){
    /*
    ** ADC conversion complete
    */
    // Read the value of ADC register and update global variable with it
    seg_value = ADCL;
    seg_value = seg_value | ADCH << 8;
    // Update 7seg
    display_one_digit(seg_value % 10, 1, 0);
    display_one_digit((seg_value / 10) % 10, 2, 0);
    display_one_digit((seg_value / 100) % 10, 3, 0);
    display_one_digit((seg_value / 1000) % 10, 4, 0);
    // Clear the ADIF flag with logical 1 to disable pending interrupts
    ADCSRA |= (1 << ADIF);
    // Start a new conversion
    start_conversion();
}

/*
** -----------------------------------------------------------------------------
** Program functions
** -----------------------------------------------------------------------------
*/
void    boot_sequence(void){

    // Display all segments in 7 segments during
    seg_value = 8888;
    seg_dot = 1;
    timer_0_conf(2, 1024, 0xFF, 0);
    // Turn on all leds from D1 to D4
    for (uint8_t i = 0; i < nb_leds; i++) {
        DDRB |= (1 << leds[i]); // Outputs
        PORTB |= (1 << leds[i]); // Turn on
    }

    // Wait 3 seconds
    my_wait(200);
    stop_timer_2();

    // Turn all off
    stop_timer_0();
    clear_all_digits();
    for (uint8_t i = 0; i < nb_leds; i++) {
        PORTB &= ~(1 << leds[i]); // Turn on
    }

    // Wait 1 second
    _delay_ms(1000);
}

void    switches_conf(void){

    // Init switches
    switch1_init(0); // Any logical change
    switch2_init(); // Any logical change by default

    // Set SW1 and SW2 as input
    for (uint8_t i = 0; i < nb_switchs; i++) {
        DDRD &= ~(1 << switchs[i]); // Inputs
        PORTD |= (1 << switchs[i]); // Helps a little bit with bouncing
    }
}

/*
** -----------------------------------------------------------------------------
** MODES
** -----------------------------------------------------------------------------
*/
void    mode0(void){
    // Enable ADC
    adc_init(10);
    // Select device
    select_device(RV1);
    start_conversion();
}

void    mode1(void){
    // Enable ADC
    adc_init(10);
    // Select device
    select_device(LDR);
    start_conversion();
}

void    mode2(void){
    // Enable ADC
    adc_init(10);
    // Select device
    select_device(NTC);
    start_conversion();
}

void    mode3(void){
    // Enable ADC
    adc_init(10);
    // Select device
    select_device(TEMP);
    start_conversion();
}

void    mode4(void){
    // print -42- on 7seg
    seg_value = 420;
    sign[0] = DASH;
    sign[3] = DASH;
    timer_0_conf(2, 1024, 0xFF, 0);

    // Blink D5, D6, D7, D8 every sec
    selected_color = 0;
    spi_master_init(); // for d6, d7, d8
    init_rgb();
    timer_1_conf(4, 256, 0, 1); // To blink and rotate leds color
}

void    mode5(void){
    
}

void    mode6(void){
}

void    mode7(void){
}

void    mode8(void){
}

void    mode9(void){
}

void    mode10(void){
}

/*
** -----------------------------------------------------------------------------
** MAIN program
** -----------------------------------------------------------------------------
*/
int main(void){

    uint8_t sw3_state;

    /*
    ** -------------------------------------------------------------------------
    ** INIT
    ** -------------------------------------------------------------------------
    */
    // UART
    uart_init(UART_BAUDRATE, 0);
    // I2C + expander
    i2c_init();
    init_i2e_expander(0b00000001, 0b00000000);
    if (i2c_error) { return (1); }
    // Interrupts
    SREG |= (1 << 7);
    timer_2_conf(2 , 64, 124, 0);

    /*
    ** -------------------------------------------------------------------------
    ** BOOT
    ** -------------------------------------------------------------------------
    */
    switches_conf();
    boot_sequence();

    // Launch mode 0
    modes[mode]();

    // Loop
    while (1) {
        // If SW3 is pressed
        sw3_state = switch3_check();
        if (sw3_state == 0){ // SW3 is pressed
            turn_on_i2c_led(D11);
            _delay_ms(DEBOUNCING);
        } else if (sw3_state == 1) {
            turn_off_i2c_led(D11);
        } else {
            // I2C error
        }
    }

    return (0);
}
