
#ifndef __LCD_TOUCH_H_
#define __LCD_TOUCH_H_

#ifdef __cplusplus
extern "C" {
#endif
	
#include "main.h"

#define READ_TIMES  5    //Number of readings
#define LOST_VAL    1    //Discard value

#define TP_ERR_RANGE 50  //tolerance scope

//Touch
#define TP_CS_0     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET)
#define TP_CS_1     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET)

#define GET_TP_IRQ    HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)

#define GET_TP_BUSY    HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)

#define TP_PRESS_DOWN    0x80      //Touch screen had pressed  
#define TP_CATH_PRES     0x40     //button has been pressed

typedef struct 
{
  uint16_t x[5];    
  uint16_t y[5];    //Use x[0], y[0] to represent the coordinates of the touch screen during this scan, 
                    //and use x[4], y[4] to store the coordinates of the first press
                          
  uint8_t  statu;        //The state of the pen
                         //b7:Press 1/Release 0 
                         //b6:0, no button pressed; 1. Press a button
////////////Touch screen calibration parameters///////////                
  float xfac;         
  float yfac;
  short xc;           //Physical value of center X coordinate (AD value)
  short yc;           //Physical value of center Y coordinate (AD value)
  uint16_t paint_color;
}tp_dev_t;

extern tp_dev_t tp_dev;  

//
void delayMicroseconds(uint32_t us);
void TP_Init(void);
uint8_t TP_Scan(uint8_t mode);
void TP_Calibrate(void);
void Load_Touch_Draw(void);
void TP_test(void);

#ifdef __cplusplus
}
#endif

#endif





