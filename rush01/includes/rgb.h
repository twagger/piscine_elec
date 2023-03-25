#ifndef RGB_H
# define RGB_H

/*
** -----------------------------------------------------------------------------
** Params
** -----------------------------------------------------------------------------
*/
# include <avr/io.h>
// PINS
#define DEL_RED PD5
#define DEL_GREEN PD6
#define DEL_BLUE PD3
#define RED 0xFF0000
#define GREEN 0x00FF00
#define BLUE 0x0000FF
#define YELLOW 0xFFFF00
#define CYAN 0x00FFFF
#define MAGENTA 0xFF00FF
#define WHITE 0xFFFFFF

/*
** -----------------------------------------------------------------------------
** Functions
** -----------------------------------------------------------------------------
*/
void    init_rgb(void);
void    display_rgb(uint32_t rgb);

#endif