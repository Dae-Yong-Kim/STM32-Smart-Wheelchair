#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "lcd_gui.h"
#include "lcd_driver.h"

#define FONT_BACKGROUND  0xffff

//set pixel color x:1~480 y:1~320
void Gui_set_pix(uint16_t x, uint16_t y, uint16_t color)
{
    if(x>lcddev.width || y>lcddev.height)
    {
        return ;
    }
    Draw_Point( x, y, color);
}

void Gui_draw_point(uint16_t x, uint16_t y, uint16_t color, uint16_t width)
{
    uint16_t xd, yd;
    if(x>lcddev.width || y > lcddev.height)
    {
        return ;
    }
    for(xd = 0;xd<width; xd++)
    {
        for(yd=0;yd<width; yd++)
        {
            if(x+xd>lcddev.width || y+yd>lcddev.height )
            {
               continue;    
            }
            Gui_set_pix(x+xd-1, y+yd-1, color);     
        }
    }
}

//Bresenham algorithm draw line
void Gui_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color, uint16_t width, LINE_STYLE type)
{
    uint16_t x = x1;
    uint16_t y = y1;
    int dx = x2>=x1?x2-x1:x1-x2;
    int dy = y2>=y1?y2-y1:y1-y2;
    int p = dx-dy;
    int e2;
    int xdir = x1>x2?-1:1;
    int ydir = y1>y2?-1:1;
    int dot_cnt = 0;
    for(;;)
    {
        dot_cnt++;
        if(type==DOTTED && dot_cnt%3==0)
        {
            Gui_draw_point(x, y, color, width);
        }
        else if(type==SOLID)
        {
            Gui_draw_point(x, y, color, width);
        }
        if(x==x2 && y==y2)
        {
            break;
        }
        e2 = 2*p;
        if(e2>= -dy)
        {
            p -= dy;
            x += xdir;
        }
        if(e2<=dx)
        {
            p += dx;
            y += ydir;
        }
    }
}

void Gui_draw_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color, uint16_t width, FILL fill)
{
    uint16_t i;
    if(x1>lcddev.width || y1 > lcddev.height || x2 > lcddev.width || y2 > lcddev.height)
    {
        printf("ERR:line point out of screen area!\r\n");
        return ;
    } 
    if(fill == FULL)
    {
        for(i=y1;i<y2;i++)
        {
            Gui_draw_line(x1, i, x2, i, color, width, SOLID);
        }
    }
    else if(fill==EMPTY)
    {
        Gui_draw_line(x1, y1, x2, y1, color, width, SOLID);
        Gui_draw_line(x1, y1, x1, y2, color, width, SOLID);
        Gui_draw_line(x1, y2, x2, y2, color, width, SOLID);
        Gui_draw_line(x2, y1, x2, y2, color, width, SOLID);
    }
}


//Bresenham  algorithm draw circle
void Gui_draw_circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color, uint16_t width, FILL fill)
{
    uint16_t dx=0,dy=r;
    int d = 1-r;
    uint16_t i;
    if(x>lcddev.width || y > lcddev.height)
    {
        printf("ERR:circle center point out of screen area!\r\n");
        return ;
    } 
    while(dy>dx)
    {
        if(fill==EMPTY)
        {
            Gui_draw_point(x+dx, y+dy, color, width);
            Gui_draw_point(x+dy, y+dx, color, width);
            Gui_draw_point(x-dx, y+dy, color, width);
            Gui_draw_point(x-dy, y+dx, color, width);
            Gui_draw_point(x-dx, y-dy, color, width);
            Gui_draw_point(x-dy, y-dx, color, width);
            Gui_draw_point(x+dx, y-dy, color, width);
            Gui_draw_point(x+dy, y-dx, color, width);
        }
        else if(fill==FULL)
        {
            for(i=dx;i<=dy;i++)
            {
                Gui_draw_point(x+dx, y+i, color, width);
                Gui_draw_point(x+i, y+dx, color, width);
                Gui_draw_point(x-dx, y+i, color, width);
                Gui_draw_point(x-i, y+dx, color, width);
                Gui_draw_point(x-dx, y-i, color, width);
                Gui_draw_point(x-i, y-dx, color, width);
                Gui_draw_point(x+dx, y-i, color, width);
                Gui_draw_point(x+i, y-dx, color, width);                
            }
        }
        if(d<0)
        {
            d += 2*dx+3;    
        }
        else
        {
            d += 2*(dx-dy)+5;
            dy--;
        }
        dx++;
    }
}


void Gui_draw_char(uint16_t x, uint16_t y, const char str_char, FONT* Font, uint16_t color, uint16_t background)
{
    uint16_t row, col_bit;

    if (x > lcddev.width || y > lcddev.height) 
    {
        printf("ERR:draw char point out of screen area!\r\n");
        return;
    }
    //the number of bytes occupied by a character
    uint16_t char_nbytes = Font->h * (Font->w / 8 + (Font->w % 8 ? 1 : 0));

    //the offset of the character
    uint32_t offset = (str_char - ' ') * char_nbytes;
    const unsigned char *ptr = &Font->table[offset];
    for (row = 0; row < Font->h; row ++) 
    {
        for (col_bit = 0; col_bit < Font->w; col_bit++ )
        {
            if (FONT_BACKGROUND == background) 
            { 
                if (*ptr & (0x80 >> (col_bit % 8)))
                {
                    Gui_set_pix(x+col_bit, y+row, color);
                }
            } 
            else 
            {
                if (*ptr & (0x80 >> (col_bit % 8))) 
                {
                    Gui_set_pix(x+col_bit, y+row, color);
                } 
                else 
                {
                    Gui_set_pix(x+col_bit, y+row, background);
                }
            }            
            if (col_bit % 8 == 7) //next byte
            {
                ptr++;
            }
        }
        if(Font->w % 8 != 0)// next line
        {
            ptr++;
        }
    }
}

void Gui_draw_str(uint16_t x, uint16_t y, const char *str_char, FONT* Font, uint16_t color, uint16_t background)
{
    uint16_t dx = x,dy = y;
    while(*str_char != '\0')
    {
        if((dx+Font->w)>lcddev.width)//wrap the line if the x direction has reached the boundary
        {
            dx = x;
            dy += Font->h;
        }
        if((dy+Font->h)>lcddev.height)//wrap the line if the y direction has reached the boundary
        {
            dy = y;
            dx += Font->w;
        }
        Gui_draw_char(dx, dy, *str_char++, Font, color, background);
        dx += Font->w;
    }

}




