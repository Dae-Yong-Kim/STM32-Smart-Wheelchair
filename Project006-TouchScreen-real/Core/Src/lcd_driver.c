#include <stdio.h>
#include "lcd_driver.h"

extern UART_HandleTypeDef huart2;

extern TIM_HandleTypeDef htim3;

//LCD pen color and background color   
uint16_t POINT_COLOR=0x0000; //brush color
uint16_t BACK_COLOR=0xFFFF;  //background color  
_lcd_dev lcddev;

extern SPI_HandleTypeDef hspi1;

void Lcd_Gpio_Init(void)
{

    LCD_CS_1;                   /* LCD  ѡ   */
}

//Hardware reset
static void Lcd_Reset(void)
{
      //LCD reset
      LCD_RST_1;
      HAL_Delay(1);
      LCD_RST_0;
      HAL_Delay(80);
      LCD_RST_1;
      HAL_Delay(10);
}

//the value is between 0 and 1000
//The frequency of PWM is 100Hz,and the duty cycle of PWM is determined by value
void BlackLight_SetValue(uint16_t value)
{

    TIM_OC_InitTypeDef sConfigOC;

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = value;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  
}

//Write command
static void Spi_write_cmd(uint8_t data)  
{ 
   LCD_CS_0;
   LCD_DC_0;
    HAL_SPI_Transmit(&hspi1,&data,1,0xff);
   LCD_CS_1;
}
//Write a byte data
static void Spi_write_data(uint8_t data) 
{ 
   LCD_CS_0;
   LCD_DC_1;
   HAL_SPI_Transmit(&hspi1,&data,1,0xff);
   LCD_CS_1;
}  
//Write a word data
void Spi_write_word(uint16_t data)
{
    uint8_t temp;
    temp = data >> 8;
    HAL_SPI_Transmit(&hspi1,&temp,1,0xff);
    temp = data & 0xff;
    HAL_SPI_Transmit(&hspi1,&temp,1,0xff);
}   

//Common register initialization
void Lcd_Init(void)
{
  Lcd_Reset();
  HAL_Delay(120);
  Spi_write_cmd(0x11);
  HAL_Delay(120);
  Spi_write_cmd(0xf0);
  Spi_write_data(0xc3);
  
  Spi_write_cmd(0xf0);
  Spi_write_data(0x96);
  
  Spi_write_cmd(0x36);
  Spi_write_data(0x48);
  
  Spi_write_cmd(0xe8);
  Spi_write_data(0x40);
  Spi_write_data(0x82);
  Spi_write_data(0x07);
  Spi_write_data(0x18);
  Spi_write_data(0x27);
  Spi_write_data(0x0a);
  Spi_write_data(0xb6);
  Spi_write_data(0x33);

  Spi_write_cmd(0xc2);
  Spi_write_data(0xa7);
  Spi_write_cmd(0xc5);
  Spi_write_data(0x25);//13
  
  Spi_write_cmd(0xb7);
  Spi_write_data(0x46);//13  06
  
  Spi_write_cmd(0xe0);
  Spi_write_data(0xf0);
  Spi_write_data(0x07);
  Spi_write_data(0x11);
  Spi_write_data(0x10);
  Spi_write_data(0x11);
  Spi_write_data(0x0b);
  Spi_write_data(0x3c);
  Spi_write_data(0x43);
  Spi_write_data(0x4b);
  Spi_write_data(0x0b);
  Spi_write_data(0x17);
  Spi_write_data(0x16);
  Spi_write_data(0x1b);
  Spi_write_data(0x1d);

  Spi_write_cmd(0xe1);
  Spi_write_data(0xf0);
  Spi_write_data(0x07);
  Spi_write_data(0x11);
  Spi_write_data(0x0f);
  Spi_write_data(0x10);
  Spi_write_data(0x1a);
  Spi_write_data(0x39);
  Spi_write_data(0x55);
  Spi_write_data(0x4a);
  Spi_write_data(0x0b);
  Spi_write_data(0x16);
  Spi_write_data(0x15);
  Spi_write_data(0x1a);
  Spi_write_data(0x1d);
  
  Spi_write_cmd(0xf0);
  Spi_write_data(0x3c);
  
  Spi_write_cmd(0xf0);
  Spi_write_data(0x69);

  Spi_write_cmd(0x3A); // Interface Pixel Format 
  Spi_write_data(0x55); 
  
  HAL_Delay(120);
  Spi_write_cmd(0x29);

  LCD_Display_Dir(DFT_SCAN_DIR); 
  BlackLight_SetValue(800);   //Turn on the backlight
  LCD_Clear(BACKGROUND_COLOR);
  printf("LCD Finish Init\r\n"); 
//
}

//Set the auto scan direction of the LCD
//Note: Other functions may be affected by this function setting
//dir:0~7,Represents 8 directions (see lcd.h for specific definitions)      
void LCD_Scan_Dir(uint8_t dir)
{
  uint16_t regval=0;
  switch(dir)
  {
    case L2R_U2D://left to right, up to down
      regval = (0<<7)|(0<<6)|(0<<5);
      break;
    case L2R_D2U://left to right, from down to up
      regval|=(1<<7)|(0<<6)|(0<<5); 
      break;
    case R2L_U2D://right to left, up to down
      regval|=(0<<7)|(1<<6)|(0<<5); 
      break;
    case R2L_D2U://right to left,down to up
      regval|=(1<<7)|(1<<6)|(0<<5); 
      break;   
    case U2D_L2R://up to down,left to right
      regval|=(0<<7)|(0<<6)|(1<<5); 
      break;
    case U2D_R2L://up to down,right to left,
      regval|=(0<<7)|(1<<6)|(1<<5);  
      break;
    case D2U_L2R://down to up,left to right
      regval|=(1<<7)|(0<<6)|(1<<5);
      break;
    case D2U_R2L://down to up,right to left,
      regval|=(1<<7)|(1<<6)|(1<<5); 
      break;   
  }
  regval|=0x08; 
  Spi_write_cmd(0x36); 
  Spi_write_data(regval);   
}

//Set the LCD display direction
//dir:vertical screen or horizontal screen
void LCD_Display_Dir(uint8_t dir)
{
   if(dir==0 || dir==1 || dir==2 || dir==3)//vertical screen
   {
      lcddev.dir=0;
      lcddev.width=LCD_H;
      lcddev.height=LCD_W;

      lcddev.wramcmd=0X2C;
      lcddev.setxcmd=0X2A;
      lcddev.setycmd=0X2B;  
   }
   else         //horizontal screen
   {           
      lcddev.dir=1; 
      lcddev.width=LCD_W;
      lcddev.height=LCD_H;

      lcddev.wramcmd=0X2C;
      lcddev.setxcmd=0X2A;
      lcddev.setycmd=0X2B;      
   } 
   LCD_Scan_Dir(dir); //default LCD display direction
}

//select the srceen area(from point(start_x,start_y) to point(end_x,end_y))
void Set_Window(uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t  end_y)
{ 
  Spi_write_cmd(0x2a);
  Spi_write_data(start_x >>8);
  Spi_write_data(start_x & 0xff);
  Spi_write_data((end_x-1) >> 8);
  Spi_write_data((end_x-1) & 0xff);

  Spi_write_cmd(0x2b);
  Spi_write_data(start_y >>8);
  Spi_write_data(start_y & 0xff);
  Spi_write_data((end_y-1) >> 8);
  Spi_write_data((end_y-1) & 0xff);

  Spi_write_cmd(0x2C);
}


//set cursor to the point(x, y)
void LCD_Set_Cursor(uint16_t x, uint16_t y)
{ 
   Set_Window(x, y, x, y);
}
//
//clear screen function
//color:fill color to clear

void SPI_Set_Speed(uint32_t speed)
{
    hspi1.Init.BaudRatePrescaler = speed;
	  if (HAL_SPI_Init(&hspi1) != HAL_OK)
		{
			Error_Handler();
		}
}

void LCD_Clear(uint16_t color)
{
  uint32_t i; 
  
  LCD_Set_Cursor(0x00,0x0000); //set cursor position
  Spi_write_cmd(lcddev.wramcmd);//Start writing to GRAM 
  LCD_CS_0;
  LCD_DC_1;
  SPI_Set_Speed(SPI_BAUDRATEPRESCALER_4);
  for(i=0;i<(uint32_t)LCD_W*LCD_H;i++)
  {
     Spi_write_word(color); 
  }
  LCD_CS_1;
   SPI_Set_Speed(SPI_BAUDRATEPRESCALER_16);
    Draw_Battery(80);
    // 예시 호출
    // 예시 호출: (x=10, y=10) 위치에 하트와 심박수 75를 표시
    Gui_draw_heart_with_heartbeat(30, 30,RED,BLACK, 0xffff, 75);
}

void LCD_Clearx(void)
{
  uint32_t i=0;      

  LCD_Set_Cursor(0,0x0000); //set cursor position
  Spi_write_cmd(lcddev.wramcmd);//Start writing to GRAM 
  LCD_CS_0;
  LCD_DC_1;     
  for(i=0;i<51200;i++)
  {
    Spi_write_word(RED);    
  }
  for(;i<102400;i++)
  { 
    Spi_write_word(GREEN);     
  }
  for(;i<153600;i++)
  {
    Spi_write_word(BLUE);      
  }
  LCD_CS_1;
}

void Gui_fill_color(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint16_t i,temp=0;
    if(x1>x2)
    {
       temp = x1;
       x1 = x2;
       x2 = temp;
    }
    if(y1>y2)
    {
       temp = y1;
       y1 = y2;
       y2 = temp;
    }
    Set_Window(x1, y1, x2, y2);
    Spi_write_cmd(lcddev.wramcmd);//Start writing to GRAM 
    LCD_CS_0;
    LCD_DC_1;      
    for(i=0; i<(x2-x1)*(y2-y1); i++)
    {
       Spi_write_word(color); 
    }
    LCD_CS_1;
}

//set the point(x,y) to color
void Draw_Point(uint16_t x, uint16_t y, uint16_t color)
{
   LCD_Set_Cursor(x, y);
   Spi_write_cmd(lcddev.wramcmd);
   LCD_CS_0;
   LCD_DC_1;
   Spi_write_word(color); 
   LCD_CS_1;  
}













