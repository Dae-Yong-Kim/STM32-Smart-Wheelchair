/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define BUF_SIZE 100
double FF_dist = 0, FR_dist = 0, FL_dist = 0, BB_dist = 0;
//0 : Slow Stop , 1 : Emergency Stop, 2 : Forward, 3: Backward
//4 : Slow Right, 5 : Quick Right
//6 : Slow Left, 7 : Quick Left
void Motor_Mode(int x)
{
	switch(x) {
		  case 0:		// Slow Stop
			  if(htim1.Instance->CCR1 > 0)
			  {
				  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
				  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
				  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
				  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
				  for(int spd = 300; spd > 0;)
				  {
					  htim1.Instance->CCR1 = spd;
					  htim1.Instance->CCR3 = spd;
					  spd -= 40;
					  HAL_Delay(100);
				  }
				  htim1.Instance->CCR1 = 0;
				  htim1.Instance->CCR3 = 0;
			  }
			  else
			  {
				  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
				  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);
				  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
				  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
			  }
			  break;
		  case 1:		// Emergency Stop
			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
			  break;
		  case 2:		// Forward
			  htim1.Instance->CCR1 = 300;
			  htim1.Instance->CCR3 = 300;

			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
			  break;
		  case 3:		// Backward
			  htim1.Instance->CCR1 = 300;
			  htim1.Instance->CCR3 = 300;
			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
			  break;
		  case 4:		// Slow Right
			  htim1.Instance->CCR1 = 450;		//ENA
			  htim1.Instance->CCR3 = 220;		//ENB

			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
			  break;
		  case 5:		// Quick Right
			  htim1.Instance->CCR1 = 300;
			  htim1.Instance->CCR3 = 300;

			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
			  break;
		  case 6:		// Slow Left
			  htim1.Instance->CCR1 = 220;		//ENA
			  htim1.Instance->CCR3 = 450;		//ENB

			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
			  break;
		  case 7:		// Quick Left
			  htim1.Instance->CCR1 = 300;
			  htim1.Instance->CCR3 = 300;

			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
			  break;
	}
}

int mv_mode = 1;
// 0 : standby, 1 : Passenger, 2 : watchdog
// 3 : Obstacle, 4 : Forward First, 5 : Right First, 6 : Left First
// 7 : Destination, 8 : Charge
void Move_mode()
{
	switch(mv_mode)
	{
	case 0 :	// Standby
		printf("======Standby========\r\n");
		Motor_Mode(1);	// Emergency Stop
		break;
	case 1 :	// Passenger
		printf("======Passenger========\r\n");
		Motor_Mode(1);	// Emergency Stop
		break;
	case 2 : 	// Watchdog
		printf("======Watchdog========\r\n");
		Motor_Mode(1);	// Emergency Stop
		break;
	case 3 :	// Obstacle
		printf("======Obstacle========\r\n");
		Motor_Mode(1);	// Emergency Stop
		break;
	case 4 :	// Forward First
		if((FR_dist < 500) && (FL_dist < 500) && (FF_dist < 500))
		{
			Motor_Mode(0);	// Slow Stop
			mv_mode = 3;
		}
		else if(FR_dist < 300)
		{
			while(FF_dist < 800)
			{
				Motor_Mode(7);	// Quick Left
				HAL_Delay(1);
			}
		}
		else if(FL_dist < 300)
		{
			while(FF_dist < 800)
			{
				Motor_Mode(5);	// Quick Right
				HAL_Delay(1);
			}
		}
		else if(FR_dist < 500)
		{
			while(FF_dist < 800)
			{
				Motor_Mode(6);	// Slow Left
				HAL_Delay(1);
			}
		}
		else if(FL_dist < 500)
		{
			while(FF_dist < 800)
			{
				Motor_Mode(4);	// Slow Right
				HAL_Delay(1);
			}
		}
		else
		{
			Motor_Mode(2);	// Forward
		}
		break;
	case 5 : 	// Right First

		break;
	case 6 :	// Left First
		printe("======Left First========");
		if((FL_dist < 150) || (FR_dist < 150) || (FF_dist < 150))
		{
			while(!printe("=============1============="));
			Motor_Mode(0);	// Slow Stop
			HAL_Delay(100);
			Motor_Mode(3);	// Backward
			HAL_Delay(10);
		}
		else if((FL_dist < 300) && (FR_dist > 500))
		{
			while(!printe("=============2============="));
			Motor_Mode(5);	// Quick Right
			HAL_Delay(2);
		}
		else
		{
			while(!printe("=============3============="));
			Motor_Mode(6);	// Slow Left
			HAL_Delay(2);
		}
		break;
	case 7 :	// Destination
		//Gyro_ModuleSet();
		Motor_Mode(2);	// Forward
		printe("======Destination========");
		while(1)
		{
			//printf("FF_dist = %.2f\r\n", FF_dist);
			if(FF_dist > 800)
			{
				Motor_Mode(2);	// Forward
			}
			else	break;
		}
		//Gyro_reSet();
		double Dturn;
		char debug_buf2[100];
		while(Dturn < 160)
		{
			//printf("BB_dist = %.2f\r\n", BB_dist);
			Read_Z_Angle(&Dturn);
			sprintf(debug_buf2, "%.2f", Dturn);
			printe(debug_buf2);
			HAL_Delay(200);
			Motor_Mode(5);		// Quick Right
			HAL_Delay(10);
		}
		while(1)
		{
			//printf("BB_dist = %.2f\r\n", BB_dist);
			//printf("backward\r\n");
			Motor_Mode(3);		// Backward
			if(BB_dist < 300)	break;
		}
		mv_mode = 1;
		//printf("curr_state : %d\r\n", mv_mode);
		break;
	case 8 :	// Charge

		break;
	}
}

// GPIO Interrupt
int testM = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin) {
	case GPIO_PIN_13:
		if(mv_mode == 1)	mv_mode = 6;
		break;
  }
}

int fF_t1 = 0, fF_t2 = 0;
int fR_t1 = 0, fR_t2 = 0;
int fL_t1 = 0, fL_t2 = 0;
int bB_t1 = 0, bB_t2 = 0;
uint8_t fF_Flag = 0;
uint8_t bB_Flag = 0;
uint8_t fR_Flag = 0;
uint8_t fL_Flag = 0;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	// Ultrasonic TIM_IC
  if (htim->Instance == TIM3)
  {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)	// FF_Echo
    {
      if (fF_Flag == 0)  // Rising Edge
      {
    	  fF_t1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    	  // Capture state
    	  fF_Flag = 1;

        // set Falling Edge Detect Mode
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
      }
      else if (fF_Flag == 1)  // Falling Edge
      {
        fF_t2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        uint32_t max_cnt = __HAL_TIM_GET_AUTORELOAD(htim);	// Timer MAX Value

        // Distance
        // (t us * 340 m/s) / 2 -> t * 0.17 mm/us
        if (fF_t2 > fF_t1)
        {
          FF_dist = ((fF_t2 - fF_t1) * 17) / 100.0;
        }
        else
        {
        	FF_dist = (((max_cnt - fF_t1) + fF_t2 + 1) * 17) / 100.0;
        }

        // Capture State initial
        fF_Flag = 0;

        // Set Rising Edge Detect Mode
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
        //__HAL_TIM_SET_COUNTER(htim, 0); // Timer Counter Initial
      }
    }
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)	// BB_Echo
    {
	  if (bB_Flag == 0)  // Rising Edge
	  {
		  bB_t1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		  // Capture state
		  bB_Flag = 1;

		// set Falling Edge Detect Mode
		__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
	  }
	  else if (bB_Flag == 1)  // Falling Edge
	  {
		bB_t2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		uint32_t max_cnt = __HAL_TIM_GET_AUTORELOAD(htim);	// Timer MAX Value

		// Distance
		// (t us * 340 m/s) / 2 -> t * 0.17 mm/us
		if (bB_t2 > bB_t1)
		{
		  BB_dist = ((bB_t2 - bB_t1) * 17) / 100.0;
		}
		else
		{
			BB_dist = (((max_cnt - bB_t1) + bB_t2 + 1) * 17) / 100.0;
		}

		// Capture State initial
		bB_Flag = 0;

		// Set Rising Edge Detect Mode
		__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
		//__HAL_TIM_SET_COUNTER(htim, 0); // Timer Counter Initial
	  }
	}
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)	// FR_Echo
	{
	  if (fR_Flag == 0)  // Rising Edge
	  {
		  fR_t1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
		  // Capture state
		  fR_Flag = 1;

		// set Falling Edge Detect Mode
		__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
	  }
	  else if (fR_Flag == 1)  // Falling Edge
	  {
		fR_t2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
		uint32_t max_cnt = __HAL_TIM_GET_AUTORELOAD(htim);	// Timer MAX Value

		// Distance
		// (t us * 340 m/s) / 2 -> t * 0.17 mm/us
		if (fR_t2 > fR_t1)
		{
		  FR_dist = ((fR_t2 - fR_t1) * 17) / 100.0;
		}
		else
		{
			FR_dist = (((max_cnt - fR_t1) + fR_t2 + 1) * 17) / 100.0;
		}

		// Capture State initial
		fR_Flag = 0;

		// Set Rising Edge Detect Mode
		__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
		//__HAL_TIM_SET_COUNTER(htim, 0); // Timer Counter Initial
	  }
	}
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)	// FR_Echo
	{
	  if (fL_Flag == 0)  // Rising Edge
	  {
		  fL_t1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
		  // Capture state
		  fL_Flag = 1;

		// set Falling Edge Detect Mode
		__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
	  }
	  else if (fL_Flag == 1)  // Falling Edge
	  {
		fL_t2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
		uint32_t max_cnt = __HAL_TIM_GET_AUTORELOAD(htim);	// Timer MAX Value

		// Distance
		// (t us * 340 m/s) / 2 -> t * 0.17 mm/us
		if (fL_t2 > fL_t1)
		{
		  FL_dist = ((fL_t2 - fL_t1) * 17) / 100.0;
		}
		else
		{
			FL_dist = (((max_cnt - fL_t1) + fL_t2 + 1) * 17) / 100.0;
		}

		// Capture State initial
		fL_Flag = 0;

		// Set Rising Edge Detect Mode
		__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
		//__HAL_TIM_SET_COUNTER(htim, 0); // Timer Counter Initial
	  }
	}
  }
}

char buf1[BUF_SIZE], buf2[BUF_SIZE]; // DMA Buffer
char dum1, dum2;
int head1 = 0, head2 = 0, tail1 = 0, tail2 = 0; // mode 0: AT command, 1: regularly send AT+INQ and detect entered slave address
int sn; //slave number
int TCP_connect = 1, WIFI_connect = 1;

// 8E4591 : Start Point, 15DA51 : Corner Point
// 37826F : Destination Point, A7EF18 : Charging State Point
char* slave_addr[5] = {"8E4591", "15DA51", "37826F", "A7EF18", "9B0C60"}; // slave address

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   if(huart == &huart6)
   {
      if((dum1 != '\r') && (dum1 != '\n')) {
         buf1[tail1++] = dum1;
         HAL_UART_Transmit(&huart2, &dum1/*== buf1+t1-1*/, 1, 10);      // putty print
      }

      if(dum1 == '\n')
      {
         if(tail1 != 0) {
           HAL_UART_Transmit(&huart2, "\r\n", 2, 10);
           buf1[tail1] = '\0';

           if(!strncmp(buf1, "4,CONNECT", 9) || !strncmp(buf1, "ALREADY CONNECTED", 17)) {
              TCP_connect = 0;
           }
           if(!strncmp(buf1, "4,CLOSED", 8)) {
              TCP_connect = 1;
           }
           if(!strncmp(buf1, "WIFI GOT IP", 11)) {
			WIFI_connect = 0;
		 }
           if(!strncmp(buf1, "WIFI DISCONNECT", 15)) {
        	   WIFI_connect = 1;
           }
        }

         tail1 = 0;
      }
      HAL_UART_Receive_IT(&huart6, &dum1, 1);         // interrupt chain
   }

   else if(huart == &huart2)
   {
      buf2[tail2++] = dum2;
      HAL_UART_Transmit(&huart2, &dum2, 1, 10); // terminal echo
      if(dum2 == '\r')  // CR : 0x0d
      {
         HAL_UART_Transmit(&huart2, "\n", 1, 10); // terminal echo

         buf2[tail2++] = '\n'; // == HAL_UART_Transmit(&huart6, "\n", 1, 10);
         HAL_UART_Transmit(&huart6, buf2, tail2, 10);   // AT Command
//         HAL_UART_Transmit(&huart6, "\n", 1, 10);
         tail2 = 0;
      }
      HAL_UART_Receive_IT(&huart2, &dum2, 1);
   }
}

// SM has to change huart6
// use : while(!printe("6100"));
int last_printe = 0;
int printe(char* str) {
   if((HAL_GetTick() - last_printe) < 100) {
      return 0;
   }
   else {
      char temp_str[BUF_SIZE], temp_AT[BUF_SIZE];
      int str_len = strlen(str);
      sprintf(temp_str, "%s", str);
      temp_str[str_len++] = '\r';
      temp_str[str_len++] = '\n';
      temp_str[str_len] = '\0';
      if((str_len > 2) && (str_len < 100)) {
         HAL_UART_Transmit(&huart6, "AT+CIPSEND=", 11, 10);
         sprintf(temp_AT, "4,%d\r\n", str_len);
         if(str_len < 10) {
            HAL_UART_Transmit(&huart6, temp_AT, 5, 10);
         }
         else {
            HAL_UART_Transmit(&huart6, temp_AT, 6, 10);
         }
         int n = str_len / 11;
         int i = 0;
         printf("loading..................................\r\n");
         for(i = 0; i < n; i++) {
            HAL_UART_Transmit(&huart6, temp_str + (i * 11), 11, 10);
         }
         HAL_UART_Transmit(&huart6, temp_str + (i * 11), str_len - (i * 11), 10);
      }
      last_printe = HAL_GetTick();
   }
   return 1;
}

void ESP8266_server_init(){
   /*HAL_UART_Transmit(&huart1, "AT+RST\r\n", 8, 10);               // RESET
   HAL_Delay(1000);*/

   HAL_UART_Transmit(&huart6, "AT+CWMODE=1", 11, 10);            // 1: CLIENT MODE, 2: SERVER MODE, 3: Multi mode
   HAL_UART_Transmit(&huart6, "\r\n", 2, 10);
   HAL_Delay(10);
   printe("111111111111111111111111111");
	  HAL_Delay(500);

//   while(WIFI_connect) { // HARMAN WIFI CONNECT
//   	  HAL_UART_Transmit(&huart6, "AT+CWJAP=\"P", 11, 10);
//   	  HAL_UART_Transmit(&huart6, "rocessor2.4", 11, 10);
//   	  HAL_UART_Transmit(&huart6, "G\",\"Process", 11, 10);
//   	  HAL_UART_Transmit(&huart6, "or1234\"\r\n", 9, 10);      // SERVER Connect
//   	  HAL_Delay(500);
//  }
//   printe("connect wifi");
//	  HAL_Delay(500);

//  while(WIFI_connect) { // SM Wifi
//	  HAL_UART_Transmit(&huart6, "AT+CWJAP=\"S", 11, 10);
//	  HAL_UART_Transmit(&huart6, "M\",\"1111111", 11, 10);
//	  HAL_UART_Transmit(&huart6, "1\"\r\n", 4, 10);	// SERVER Connect
//	  HAL_Delay(1000);
//	}
//	 printe("connect wifi");
//	  HAL_Delay(500);

   HAL_UART_Transmit(&huart6, "AT+CIPMUX=1", 11, 10);            // 0: single connection mode, 1: multi connection mode
   HAL_UART_Transmit(&huart6, "\r\n", 2, 10);
   HAL_Delay(10);

   while(TCP_connect) { // sm TCP
   	  HAL_UART_Transmit(&huart6, "AT+CIPSTART", 11, 10);
   	  HAL_UART_Transmit(&huart6, "=4,\"TCP\",\"1", 11, 10);
   	  HAL_UART_Transmit(&huart6, "92.168.221.", 11, 10);
   	  HAL_UART_Transmit(&huart6, "157\",3000\r\n", 11, 10);      // SERVER Connect
   	  HAL_Delay(1000);
  }

//   while(TCP_connect) { // HARMAN TCP SERVER CONNECT
//   	  HAL_UART_Transmit(&huart6, "AT+CIPSTART", 11, 10);
//   	  HAL_UART_Transmit(&huart6, "=4,\"TCP\",\"1", 11, 10);
//   	  HAL_UART_Transmit(&huart6, "92.168.0.65", 11, 10);
//   	  HAL_UART_Transmit(&huart6, "\",3000\r\n", 8, 10);      // SERVER Connect
//   	  HAL_Delay(500);
//  }

   printe("connect TCP server");

   /*HAL_UART_Transmit(&huart1, "AT+CIFSR", 11, 10);               // Show IP & MAC Address
   HAL_UART_Transmit(&huart1, "\r\n", 2, 10);
   HAL_Delay(10);*/
}



//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//   if(huart == &huart1)
//   {
//      buf1[tail1++] = dum1;
//      if(!mv_mode) {
//    	  HAL_UART_Transmit(&huart2, &dum1/*== buf1+t1-1*/, 1, 10);      // putty print
//      }
//      if(dum1 == '\r')
//      {
//    	  if(mv_mode){
//			  char comp_buf[BUF_SIZE];
//			  if(tail1 > 15) {
//				  sprintf(comp_buf, "%s\n\0", &buf1[tail1 - 7]);
//				  // Start Point
//				  if(!strncmp(comp_buf, slave_addr[0], 6))
//				  {
//					  // Move mode change : pasenger -> forward
//				  }
//				  // Corner Point
//				  else if(!strncmp(comp_buf, slave_addr[1], 6))
//				  {
//					  mv_mode = 6;
//				  }
//				  // Destination Point
//				  else if(!strncmp(comp_buf, slave_addr[2], 6))
//				  {
//					  mv_mode = 7;
//				  }
//				  // Charging State Point
//				  else if(!strncmp(comp_buf, slave_addr[3], 6))
//				  {
//					  mv_mode = 7;
//				  }
////				  if(!strncmp(comp_buf, slave_addr[sn], 6)) {
////					  /* Add to Mode select */
////					  printf("I found it%d\r\n\r\n", temp2++);
////				  }
//			  }
//    	  }
//         tail1 = 0;
//      }
//      HAL_UART_Receive_IT(&huart1, &dum1, 1);         // interrupt chain
//   }
////   Debugging
////   else if(huart == &huart2)
////   {
////      buf2[tail2++] = dum2;
////      HAL_UART_Transmit(&huart2, &dum2, 1, 10); // terminal echo
////      if(dum2 == '\r')  // CR : 0x0d
////      {
////         HAL_UART_Transmit(&huart2, "\n", 1, 10); // terminal echo
////         buf2[tail2++] = '\n'; // == HAL_UART_Transmit(&huart1, "\n", 1, 10);
////         HAL_UART_Transmit(&huart1, buf2, tail2, 10);   // AT Command
////         tail2 = 0;
////      }
////      HAL_UART_Receive_IT(&huart2, &dum2, 1);
////   }
//}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM4) {
		// Timer4 interrupt : 1.5 sec Period
		HAL_UART_Transmit(&huart1, "AT+INQ\r\n", 8, 10);
	}
//	else if (htim->Instance == TIM3) {
//	        // Timer3 interrupt	0.06 sec Period
//		Voltage_state();
//	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_USART6_UART_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);	// Motor PWM1
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);	// Motor PWM2
//	htim1.Instance->CCR1 = 300;
//	htim1.Instance->CCR3 = 300;
	HAL_TIM_Base_Start(&htim2);
	//UART_Start_Receive_IT(&huart1, &dum1, 1);
	UART_Start_Receive_IT(&huart2, &dum2, 1);
	UART_Start_Receive_IT(&huart6, &dum1, 1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);	// Ultrasonic FB_Trig
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);	// Ultrasonic RL_Trig
	HAL_TIM_Base_Start_IT(&htim4);			// bluetooth
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);	// Ultrasonic FF_Echo
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);	// Ultrasonic BB_Echo
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);	// Ultrasonic FR_Echo
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);	// Ultrasonic FL_Echo
	i2c_Gyro_init(&hi2c1);
	Gyro_ModuleSet();

	//ESP8266_server_init();
	ProgramStart("Motor + Bluetooth + Usonic + Gyro");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  printf("FF_dist = %.2f, FR_dist = %.2f, FL_dist = %.2f, BB_dist = %.2f\r\n", FF_dist, FR_dist, FL_dist, BB_dist);
	  Move_mode();

	  HAL_Delay(1000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8400-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 84-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim5.Init.Period = 60000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 59990-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10-1;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IN1_Pin|IN4_Pin|IN2_Pin|IN3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_Pin IN4_Pin IN2_Pin IN3_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN4_Pin|IN2_Pin|IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
