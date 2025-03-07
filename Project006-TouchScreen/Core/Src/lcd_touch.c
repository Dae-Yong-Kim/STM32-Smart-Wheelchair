#include <string.h>
#include <stdio.h>  
#include <stdlib.h>
#include <math.h>
#include "lcd_touch.h"
#include "lcd_gui.h"
#include "lcd_driver.h"


tp_dev_t tp_dev;  

void delayMicroseconds(uint32_t us)
{
   do
	 {
	     __NOP();
 }while(us--);

}


void TP_Init(void)
{
    tp_dev.xfac = -7.86F;
    tp_dev.yfac = 11.32F;
    tp_dev.xc = 2047;
    tp_dev.yc = 2047;
    tp_dev.paint_color = BLACK;    
}


///*******************************************************************************
//  function:
//        Read the ADC of the channel
//  cmd:
//    S    A2  A1  A0  MODE  SER/DFR   PD1  PD0 
//    1    0   0   1    0      0        0    0     :0X90
//    --read X+, ADC resolution is 12 bits,Differential,Power-Down Mode 

//    1    1   0   1    0      0        0    0     :0XD0
//    --read Y+, ADC resolution is 12 bits,Differential,Power-Down Mode     
//*******************************************************************************/

static uint16_t TP_read_ADC(uint8_t cmd)
{
   uint16_t data = 0;
   uint8_t tx_val=0;
	 uint8_t rx_val =0;

	 SPI_Set_Speed(SPI_BAUDRATEPRESCALER_128);
   TP_CS_0;
	 HAL_SPI_TransmitReceive(&hspi1,&cmd,&rx_val,1,0xff);
   delayMicroseconds(1000);
	 tx_val = 0xFF;
   HAL_SPI_TransmitReceive(&hspi1,&tx_val,&rx_val,1,0xff);
   data = rx_val;
   data <<= 8;
   HAL_SPI_TransmitReceive(&hspi1,&tx_val,&rx_val,1,0xff);	 
   data |= rx_val;
   data >>= 3;
   data &= 0x0FFF;
   TP_CS_1;

	 SPI_Set_Speed(SPI_BAUDRATEPRESCALER_8);
   //printf("read ADC - data :  %d , rx_val : %d\n\r", data, rx_val);
   return data;
}



/*******************************************************************************
Continuous reading of TP_ READ_ TIMES sub data, sort these data in ascending order, 
and then remove the lowest and highest TP_ LOST_ Number of VALs, taking the average value
When setting, it needs to meet the following requirements: READ_TIMES>2 * LOST_VAL conditions
cmd :   0x90 :Read channel X+
        0xd0 :Read channel Y+
*******************************************************************************/
static unsigned int TP_read_adc_avg(unsigned char cmd)
{
    unsigned char i, j;
    uint16_t adc_buff[READ_TIMES];
    unsigned int sum = 0, temp = 0;

    for (i = 0; i < READ_TIMES; i++) 
    {
        adc_buff[i] = TP_read_ADC(cmd);
        delayMicroseconds(210);
    }

    //sort the read adc data
    for (i = 0; i < READ_TIMES  -  1; i ++) 
    {
       for (j = i + 1; j < READ_TIMES; j ++) 
       {
          if (adc_buff[i] > adc_buff[j]) 
          {
             temp = adc_buff[i];
             adc_buff[i] = adc_buff[j];
             adc_buff[j] = temp;
          }
      }
   }
   //Exclude the largest and the smallest
   for (i = LOST_VAL; i < READ_TIMES - LOST_VAL; i++)
	 {
      sum += adc_buff[i];
	 }
   //get the median average value
   temp = sum / (READ_TIMES - 2 * LOST_VAL);
   return temp;
}

/*******************************************************************************
Read X channel and Y channel AD value
parameter:
x_adc,y_adc:The x and y coordinate adc value read
*******************************************************************************/
static void TP_Read_ADC_XY(uint16_t *x_adc, uint16_t  *y_adc )
{
   uint16_t a=1;//X Y and screen direction flag
   if(a)//X Y direction is opposite to the screen
   {
      *x_adc = TP_read_adc_avg(0x90);
      *y_adc = TP_read_adc_avg(0xD0);

   }
   else
   {
      *x_adc = TP_read_adc_avg(0xD0);
      *y_adc = TP_read_adc_avg(0x90);    
   }
}

///*******************************************************************************
//Read the touch screen IC twice in a row, and the deviation between these two readings 
//should not exceed TP_ERR_RANGE, if the condition is met, the reading is considered correct, 
//otherwise the reading is incorrect. This function can greatly improve accuracy
//parameter:
//x_adc,y_adc :The x and y coordinate adc value read
//*******************************************************************************/

uint8_t TP_Read_ADC_XY2(uint16_t *x_adc, uint16_t  *y_adc )
{
    uint16_t x1, y1, x2, y2;
    
    TP_Read_ADC_XY(&x1, &y1);//read first data
    TP_Read_ADC_XY(&x2, &y2);//read second data

    //The two samples before and after are within +-TP_ERR_RANGE
    if ( ((x2 <= x1 && x1 < x2 + TP_ERR_RANGE) || (x1 <= x2 && x2 < x1 + TP_ERR_RANGE))
       && ((y2 <= y1 && y1 < y2 + TP_ERR_RANGE) || (y1 <= y2 && y2 < y1 + TP_ERR_RANGE))
		   && ((x1 != 4095) && (x1 != 0) && (x2 != 4095) && (x2 != 0) && (y1 != 4095) && (y1 != 0) && (y2 != 4095) && (y2 != 0))) 
    {
//			 printf("AD x1:%d y1:%d x2:%d y2:%d\r\n", x1,y1, x2, y2);
       *x_adc = (x1 + x2) / 2;
       *y_adc = (y1 + y2) / 2;		
       return 1;
    }
    return 0;
}

//function:touch button scan
//mode: 1 : calibration; 0 : relative position
unsigned char TP_Scan(unsigned char mode)
{
  //In X, Y coordinate measurement, IRQ is disabled and output is low
  if (!GET_TP_IRQ) //Press the button to press
  {
			//Read the physical coordinates
			if (mode) 
			{
				 TP_Read_ADC_XY2(&tp_dev.x[0], &tp_dev.y[0]);
			} 
			else if (TP_Read_ADC_XY2(&tp_dev.x[0], &tp_dev.y[0])) //Read the screen coordinates
			{
					//Convert the X-axis physical coordinates into logical coordinates (that is, corresponding to the X coordinate value on the LCD screen) 
					tp_dev.x[0] = (signed short)(tp_dev.x[0] - tp_dev.xc) / tp_dev.xfac + lcddev.width / 2;

					//Convert the Y-axis physical coordinates into logical coordinates (that is, corresponding to the Y coordinate value on the LCD screen) 
					tp_dev.y[0] = (signed short)(tp_dev.y[0] - tp_dev.yc) / tp_dev.yfac + lcddev.height / 2;

			}
			if ((tp_dev.statu & TP_PRESS_DOWN) == 0) //was not pressed before 
			{
				 tp_dev.statu = TP_PRESS_DOWN | TP_CATH_PRES; //pressed 
				 tp_dev.x[4] = tp_dev.x[0]; //Save the coordinates of the first press
				 tp_dev.y[4] = tp_dev.y[0];
			}
  }
  else
  {
			if (tp_dev.statu & TP_PRESS_DOWN) //was pressed before
			{
				 tp_dev.statu &= ~TP_PRESS_DOWN; //Mark key released
			}
			else 
			{
				 tp_dev.x[4] = 0;
				 tp_dev.y[4] = 0;
				 tp_dev.x[0] = 0xffff;
				 tp_dev.y[0] = 0xffff;
			}
  }
  return (tp_dev.statu & TP_PRESS_DOWN);
}

///*******************************************************************************
//  function:
//        Draw calibration reference points
//  parameter:
//        x :    The x coordinate of the point
//        y :    The y coordinate of the point
//        color  :    Set color
//*******************************************************************************/
static void TP_Draw_Calib_Cross(uint16_t x, uint16_t y, uint16_t color)
{
   Gui_draw_line(x-10, y, x+10, y, color, 1, SOLID);
   Gui_draw_line(x, y-10, x, y+10, color, 1, SOLID);
   Gui_draw_point(x, y, color, 2);
   Gui_draw_circle(x, y, 6, color, 1, EMPTY);
}

//Prompt for calibration results (various parameters)
// xy[5][2]: 5 physical coordinate values
// px,py   : Scale factor in x, y direction (closer to 1 is better)
static void TP_calib_info_show(uint16_t pos_xy[5][2], double px, double py)
{
    uint8_t i;
    char sbuf[20];

    for (i = 0; i < 5; i++)   //show 5 physical coordinate values 
    {
        sprintf(sbuf, "x%d:%d", i + 1, pos_xy[i][0]);
        Gui_draw_str(40, 160 + (i * 20), sbuf, &Font24, RED, BACKGROUND_COLOR);
        sprintf(sbuf, "y%d:%d", i + 1, pos_xy[i][1]);
        Gui_draw_str(40+ 100, 160 + (i * 20), sbuf, &Font24, RED, BACKGROUND_COLOR);
    }
    
    //Show scale factor in X/Y direction
    Gui_fill_color(40,160 + (i * 20), lcddev.width - 1, 180 + (i * 20), BACKGROUND_COLOR);//Clear the previous px, py display
    sbuf[0] = 'p';
    sbuf[1] = 'x';
    sbuf[2] = ':';
		sprintf(sbuf+3, "%.2f", px);
    sbuf[7] = 0;//add end character
    Gui_draw_str(40, 160 + (i * 20), sbuf, &Font24, RED, BACKGROUND_COLOR);
    sbuf[0] = 'p';
    sbuf[1] = 'y';
    sbuf[2] = ':';
    sprintf(sbuf+3, "%.2f", py);


    sbuf[7] = 0;//add end character
    Gui_draw_str(40 + 100, 160 + (i * 20), sbuf, &Font24, RED, BACKGROUND_COLOR);
}

//Touch Screen Calibration Code
//Get 5 calibration parameters
void TP_Calibrate(void)
{                 
  uint16_t pos_temp[5][2];//Coordinate cache value
  short s1,s2,s3,s4;
  double px, py; //X, Y axis physical coordinate ratio, used to determine whether the calibration is successful
  uint8_t  cnt=0;  
  uint16_t outtime=0;       
 
  LCD_Clear(WHITE);//clear screen   
  POINT_COLOR=RED;
  //Show prompt information
  Gui_draw_str(30, 50, "Please use the stylus click the cross on the screen.The cross will always move until the screen adjustment is completed.", &Font24, RED, BACKGROUND_COLOR);
  TP_Draw_Calib_Cross(20, 20, RED);//draw point 1 
  tp_dev.statu=0; 
  tp_dev.xfac=0;//xfac is used to mark whether it has been calibrated, so it must be cleared before calibration! To avoid errors
  while(1)//If there is no press for 10 seconds, it will automatically exit
  {
     TP_Scan(1);//scan physical coordinates
     if((tp_dev.statu&0xc0)==TP_CATH_PRES)//The button is pressed once (the button is released at this time.)
     { 
        outtime=0;    
        tp_dev.statu&=~(1<<6);//Indicates that the button has been processed.
                         
        pos_temp[cnt][0]=tp_dev.x[0];
        pos_temp[cnt][1]=tp_dev.y[0];
        cnt++;    
        switch(cnt)
        {        
            case 1:            
              TP_Draw_Calib_Cross(20,20,WHITE);       //clear point 1 
              TP_Draw_Calib_Cross(lcddev.width-20,20,RED);  //draw point 2
              break;
            case 2:
              TP_Draw_Calib_Cross(lcddev.width-20,20,WHITE);  //clear point 2
              TP_Draw_Calib_Cross(20,lcddev.height-20,RED); //draw point 3
              break;
            case 3:
              TP_Draw_Calib_Cross(20,lcddev.height-20,WHITE);     //clear point 3
              TP_Draw_Calib_Cross(lcddev.width-20,lcddev.height-20,RED);  //draw point 4
              break;
            case 4:
              LCD_Clear(WHITE);   // Draw the fifth point, clear the screen directly
              TP_Draw_Calib_Cross(lcddev.width / 2, lcddev.height / 2, RED);  //draw point 5
              break;
            case 5:     //had got all 5 points 
              s1 = pos_temp[1][0] - pos_temp[0][0]; //The X-axis physical coordinate difference (AD value) between the 2th point and the 1th point
              s3 = pos_temp[3][0] - pos_temp[2][0]; //The X-axis physical coordinate difference (AD value) between the 4th point and the 3th point
              s2 = pos_temp[3][1] - pos_temp[1][1]; //The Y-axis physical coordinate difference (AD value) between the 4th point and the 2th point
              s4 = pos_temp[2][1] - pos_temp[0][1]; //The X-axis physical coordinate difference (AD value) between the 3th point and the 1th point

              px = (double)s1 / s3; //X-axis scale factor
              py = (double)s2 / s4; //Y-axis scale factor

              if (px < 0)px = -px; //Negative to positive
              if (py < 0)py = -py; //Negative to positive

              if (px < 0.95 || px > 1.05 || py < 0.95 || py > 1.05 ||     //Unqualified ratio
                      abs(s1) > 4095 || abs(s2) > 4095 || abs(s3) > 4095 || abs(s4) > 4095 || //The difference is unqualified, greater than the coordinate range
                      abs(s1) == 0 || abs(s2) == 0 || abs(s3) == 0 || abs(s4) == 0            //The difference is unqualified, equal to 0
                 )
              {
                  cnt = 0;
								  HAL_Delay(2000);
                  TP_Draw_Calib_Cross(lcddev.width / 2, lcddev.height / 2, WHITE); //clear point 5
                  TP_Draw_Calib_Cross(20, 20, RED);   //repaint ponit 1
								  printf("show calibrate info\r\n");
                  TP_calib_info_show(pos_temp, px, py);   //Display current information, easy to find problems
								  Gui_draw_str(30, 50, "Please use the stylus click the cross on the screen.The cross will always move until the screen adjustment is completed.", &Font24, RED, BACKGROUND_COLOR);
                  continue;
              }

              tp_dev.xfac = (float)(s1 + s3) / (2 * (lcddev.width - 40));
              tp_dev.yfac = (float)(s2 + s4) / (2 * (lcddev.height - 40));
              tp_dev.xc = pos_temp[4][0];      //X axis, physical center coordinates
              tp_dev.yc = pos_temp[4][1];      //Y axis, physical center coordinates
              LCD_Clear(WHITE);  //clear screen
              Gui_draw_str(45, 110, "Touch Screen Calibration OK!", &Font24, BLUE, BACKGROUND_COLOR);//Calibration completed
              HAL_Delay(2000);
              TP_calib_info_show(pos_temp, px, py); 
              HAL_Delay(3000);
              printf("Calibration over\r\n");
              return;//Calibration completed
          }
       }
       HAL_Delay(10);
       outtime++;
       if(outtime>1000)
       {
          printf("calibrate outtime,Init Touch Pad...\r\n");
				  TP_Init();
          break;
      } 
   }
}

void Load_Touch_Draw(void)
{
    tp_dev.paint_color = BLACK;
    //LCD_Clear(BACKGROUND_COLOR);

    Gui_draw_circle(130, 165, 65, BLACK, 1, FULL);
    Gui_draw_str(84, 155, "Dest A", &Font24, MAGENTA, BLACK);

    Gui_draw_circle(330, 165, 65, BLACK, 1, FULL);
    Gui_draw_str(280, 155, "Dest B", &Font24, YELLOW, BLACK);
   // Gui_fill_color(200, 150, 290, 90, BLUE);
    //Gui_draw_str(200, 150, "ROOM 2", &Font24, WHITE, BLUE);

/*    Gui_fill_color(5, 114, 80, 141, RED);
    Gui_fill_color(5, 156, 80, 183, GREEN);
    Gui_fill_color(5, 198, 80, 225, BLUE);
    Gui_fill_color(5, 240, 80, 267, BLACK);
    Gui_fill_color(5, 282, 80, 309, YELLOW);*/
}

void ShowSuccessMessage(void) {
    // 텍스트의 크기 계산
    uint16_t str_width = 0;
    uint16_t str_height = Font24.h;  // 폰트의 높이
    const char* str_char = "Success";

    // 텍스트 길이에 따라 너비 계산
    while(*str_char != '\0') {
        str_width += Font24.w;  // 각 글자의 너비를 더함
        str_char++;
    }

    // 중앙 정렬을 위한 좌표 계산
    uint16_t x_center = (lcddev.width - str_width) / 2;
    uint16_t y_center = (lcddev.height - str_height) / 2;

    // 중앙 정렬된 텍스트 출력
    Gui_draw_str(x_center, y_center, "Success", &Font24, GREEN, WHITE);  // 텍스트 중앙에 출력
    Gui_draw_str(370, 290, "CLEAR", &Font24, WHITE, BLUE);
}

void TP_test(void)
{
   uint16_t paint_width = 2;
   while(1)
   {
      TP_Scan(0);
      //printf("****SCAN : x: %d, y: %d\r\n", tp_dev.x[0], tp_dev.y[0]);
      HAL_Delay(1000);
      if(tp_dev.statu&TP_PRESS_DOWN)  //touch screen is pressed
      {
         if(tp_dev.x[0]<lcddev.width && tp_dev.y[0]<(lcddev.height)) //The touch point is within the area of the screen
         {
        	 LCD_Clear(BACKGROUND_COLOR);
        	 ShowSuccessMessage();
        	 printf(" x: %d, y: %d\r\n", tp_dev.x[0], tp_dev.y[0]);

         }

      }
   }
 }


#define BATTERY_X1 400
#define BATTERY_Y1 20
#define BATTERY_WIDTH 50
#define BATTERY_HEIGHT 20
#define BATTERY_BORDER_WIDTH 2

void Draw_Battery(int battery_percentage) {
    // 배터리 외형 그리기
    Gui_draw_rectangle(BATTERY_X1, BATTERY_Y1, BATTERY_X1 + BATTERY_WIDTH, BATTERY_Y1 + BATTERY_HEIGHT, BLACK, BATTERY_BORDER_WIDTH, EMPTY);

    // 배터리 충전 상태에 맞게 충전 부분 채우기
    int battery_fill_width = (battery_percentage * BATTERY_WIDTH) / 100; // 배터리 충전 상태에 따른 너비 계산
    Gui_draw_rectangle(BATTERY_X1 + BATTERY_BORDER_WIDTH, BATTERY_Y1 + BATTERY_BORDER_WIDTH, BATTERY_X1 + BATTERY_BORDER_WIDTH + battery_fill_width, BATTERY_Y1 + BATTERY_HEIGHT, GREEN, 1, FULL);

    // 배터리  용량이 20% 미만일 경우 빨간색으로 표시
    if (battery_percentage < 20) {
        Gui_draw_rectangle(BATTERY_X1 + BATTERY_BORDER_WIDTH, BATTERY_Y1 + BATTERY_BORDER_WIDTH, BATTERY_X1 + BATTERY_BORDER_WIDTH + battery_fill_width, BATTERY_Y1 + BATTERY_HEIGHT , RED, 1, FULL);
    }

/*    // 배터리 잔량을 텍스트로 표시
    char battery_text[10];
    snprintf(battery_text, sizeof(battery_text), "%d%%", battery_percentage);
   Gui_draw_str(BATTERY_X1 + BATTERY_WIDTH + 5, BATTERY_Y1 + 5, battery_text, &Font16, WHITE, BLACK);*/
}


