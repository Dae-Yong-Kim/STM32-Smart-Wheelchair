#ifndef __LCD_GUI_H
#define __LCD_GUI_H

#include <stdio.h>
#include <stdlib.h>
#include "fonts.h"
#include "lcd_driver.h"


typedef enum{
    EMPTY = 0,
    FULL,	
} FILL;

typedef enum{
    SOLID = 0,  //_______________
    DOTTED,	    //---------------
} LINE_STYLE;

typedef enum{
    ROUND = 0,             
    SQUAER,	    
} DOT_STYLE;

void Image_Init( uint16_t img_w, uint16_t img_h, uint16_t rotate, uint16_t mirror);
void Gui_set_pix(uint16_t x, uint16_t y, uint16_t color);
//void Gui_Clear(uint16_t color);
void Gui_draw_point(uint16_t x, uint16_t y, uint16_t color, uint16_t width);
void Gui_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color, uint16_t width, LINE_STYLE  type);
void Gui_draw_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color, uint16_t width, FILL fill);
void Gui_draw_circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color, uint16_t width, FILL fill);
void Gui_draw_char(uint16_t x, uint16_t y, const char str_char,FONT* Font, uint16_t color, uint16_t background);
void Gui_draw_str(uint16_t x, uint16_t y, const char *str_char, FONT* Font, uint16_t color, uint16_t background);

#endif
