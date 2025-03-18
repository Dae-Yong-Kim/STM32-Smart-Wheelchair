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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <string.h>

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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

osThreadId Task1Handle;
osThreadId Task2Handle;
osThreadId Task03Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
void StartTask1(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Gyro Max Value
//double MAX_D = 0;
int MAX_D = 0;

// SM has to change huart6
// use : while(!printe("6100"));
int last_printe = 0;
#define BUF_SIZE 100
char buf1[BUF_SIZE], buf2[BUF_SIZE], buf3[BUF_SIZE]; // DMA Buffer
char dum1, dum2, dum3;
int tail1 = 0, tail2 = 0, tail3 = 0; // 1 : ESP, 2 : Putty, 3 : Bluetooth
//int sn; //slave number
int TCP_connect = 1, WIFI_connect = 1, esp_set = 0;
//int ESP_state = 0;

/* ESP8266 Send : Destination Information
Start point1 (8E4591) = D (2100)   		: 8
Destination point2 (37826F) = A(2200)   : 6
charging Station (A7EF18) = B(2300)	: 4 */
int ESP_send = 0;

/* send to ESP : Start Point & Charging Station
Corner (15DA51) = C (1000)
Start point1 (8E4591) = D (1100)
Destination point2 (37826F) = A(1200)
charging Station (A7EF18) = B(1300)*/
int STM_send = 0;

// Driving Mode Select
/*0 : standby, 1 : Passnger, 2 : Drive, 3 : Watchdog, 4 : Destination, 5 : Charge*/
int drive_set = 0;

/* 8E4591 : Start Point, 15DA51 : Corner Point
 37826F : Destination Point, A7EF18 : Charging State Point */
char* slave_addr[5] = {"8E4591", "15DA51", "37826F", "A7EF18"/*, "9B0C60"*/};
/*point1 (8E4591) : 1
point2 (37826F) : 2
pcharge (A7EF18) : 3*/
int BLE_point = 0;
/* Map List
map = 1 : point1 - corner1 - point2 = 7
map = 2 : point1 - corner1 - pcharge = 5
map = 3 : point2 - corner1 - point1 = 10
map = 4 : point2 - corner1 - pcharge = 6
map = 5 : pcharge - corner1 - point1 = 11
map = 6 : pcharge - corner1 - point2 = 9*/
int map_select = 0;
int map_set = 0;
// Drive Mode Set
/*0 : straight, 1 : left, 2 : right, 3 : back*/
int Move_mode = 0;
void MAP_operation()
{
	switch(map_select)
	{
	case 1 :	// 8E4591 - 15DA51 - 37826F
		if(drive_set == 2)
		{
			if(BLE_point == 4)		Move_mode = 1;
			else if(BLE_point == 2)	drive_set = 4;
		}
		break;
	case 2 :	// 8E4591 - 15DA51 - A7EF18
		if((drive_set == 2) || (drive_set == 5))
		{
			if(BLE_point == 4)		Move_mode = 0;
			else if(BLE_point == 3)	drive_set = 4;
		}
		break;
	case 3 :	// 37826F - 15DA51 - 8E4591
		if(drive_set == 2)
		{
			if(BLE_point == 4)		Move_mode = 2;
			else if(BLE_point == 1)	drive_set = 4;
		}
		break;
	case 4 :	// 37826F - 15DA51 - A7EF18
		if((drive_set == 2) || (drive_set == 5))
		{
			if(BLE_point == 4)		Move_mode = 1;
			else if(BLE_point == 3)	drive_set = 4;
		}
		break;
	case 5 :	// A7EF18 - 15DA51 - 8E4591
		if(drive_set == 2)
		{
			if(BLE_point == 4)		Move_mode = 0;
			else if(BLE_point == 1)	drive_set = 4;
		}
		break;
	case 6 :	// A7EF18 - 15DA51 - 37826F
		if(drive_set == 2)
		{
			if(BLE_point == 4)		Move_mode = 2;
			else if(BLE_point == 2)	drive_set = 4;
		}
		break;
	default :
		break;
	}
}

int safe_st = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   if(huart == &huart6)
   {
	  //printf("11111111111111111111111111111111111\r\n");
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
//           if(!strncmp(buf1, "WIFI GOT IP", 11)) {
//			WIFI_connect = 0;
//		 }
//           if(!strncmp(buf1, "WIFI DISCONNECT", 15)) {
//        	   WIFI_connect = 1;
//           }
           if(!strncmp(buf1, "OK", 2)) {
        	   esp_set++;
           }
           if(!strncmp(buf1, "+IPD,", 5)) {
			 switch(buf1[9]) {
				case '0':	// Emergency & Normal State Set
				   //buf1[10] 0 : emergency, 1 : Force OK, 2 : Heart Rate OK, 3 : Total OK
					unsigned char ESP_words0 = buf1[10];
					switch(drive_set)
					{
					case 0 :
						if(ESP_words0 == '1')	drive_set = 1;	// Passenger
						else					drive_set = 0;
						break;
					case 1 :
						if(ESP_words0 == '3')	safe_st = 1;
						else					drive_set = 1;
						break;
					case 2 :
						if(ESP_words0 == '0')	drive_set = 3;	//	Watchdog
						else					drive_set = 2;
						break;
					case 3 :
						if(ESP_words0 == '3')	drive_set = 2;
						else					drive_set = 3;
						break;
					}
//					if(drive_set == 2)	// Driveing Mode
//					{
//						if(ESP_words0 == '0')	drive_set = 3;	//	Watchdog
//						else					drive_set = 2;
//					}
//					else if(drive_set == 0) // Standby Mode
//					{
//						if(ESP_words0 == '1')	drive_set = 1;	// Passenger
//						else					drive_set = 0;
//					}
//					else if(drive_set == 1)	// Passenger Mode
//					{
//						if(ESP_words0 == '3')	safe_st = 1;
//						else					drive_set = 1;
//					}
//					else if(drive_set == 3)	// Watchdog Mode
//					{
//						if(ESP_words0 == '3')	drive_set = 2;
//						else					drive_set = 3;
//					}
					break;
//				case '1':	// Origin is xxx (Not use in Server)
//				   // Origin is xxx
//				   break;
				case '2':	// Destination Set
					unsigned char ESP_words1 = buf1[10];
					if(drive_set == 1)
					{
						if(ESP_words1 == '1')		ESP_send = 8;
						else if(ESP_words1 == '2')	ESP_send = 6;
						else if(ESP_words1 == '3')	ESP_send = 4;
					}
				   break;
//				case '3':	// Arrive at Destination (Not use in Server)
//				   break;
//				case '4':	// Hall Sensor
//				   break;
				case '5':	// Force Sensor
					unsigned char ESP_words2 = buf1[10];
					switch(drive_set)
					{
					case 0 :
						if(ESP_words2 == '1')	drive_set = 1;
						else					drive_set = 0;
						break;
					case 1 :
						if(ESP_words2 == '0')	drive_set = 0;
						else					drive_set = 1;
						break;
					}
//					if(drive_set == 0)	// standby Mode
//					{
//						if(ESP_words2 == '1')	drive_set = 1;
//						else					drive_set = 0;
//					}
//					else if(drive_set == 1)	// Passenger Mode
//					{
//						if(ESP_words2 == '0')	drive_set = 0;
//						else					drive_set = 1;
//					}
				   break;
//				case '6':	// Heart Beat Sensor
//				   break;
//				case '7':	// Voltage Sensor (Not use in Server)
//				   printf("remain battery: %s%", comp_buf + 10);
//				   battery = atoi(comp_buf + 10);
//				   break;
//				case '8':	// STM Client
//				   break;
			 }
		  }
        }

         tail1 = 0;
      }
      HAL_UART_Receive_IT(&huart6, &dum1, 1);         // interrupt chain

   }
   else if(huart == &huart1)
	{
	 buf3[tail3++] = dum3;
	 if(1) {
	  HAL_UART_Transmit(&huart2, &dum3/*== buf1+t1-1*/, 1, 10);      // putty print
	 }
	 if(dum3 == '\r')
	 {
	  if(drive_set != 0){
		  char comp_buf[BUF_SIZE];
		  if(tail3 > 15) {
			  sprintf(comp_buf, "%s\n\0", &buf3[tail3 - 7]);
			  // Start Point
			  if(!strncmp(comp_buf, slave_addr[0], 6))	// 8E4591 ( Start Point )
			  {
				  BLE_point = 1;
				  MAP_operation();
				  //printf("========Start BLE %d========\r\n", BLE_point);
			  }
			  // Corner Point
			  else if(!strncmp(comp_buf, slave_addr[1], 6))	// 15DA51 ( Corner Point )
			  {
				  BLE_point = 4;
				  MAP_operation();
				  //printf("========Corner BLE %d========\r\n", BLE_point);
			  }
			  // Destination Point
			  else if(!strncmp(comp_buf, slave_addr[2], 6))	// 37826F ( Destination Point )
			  {
				  BLE_point = 2;
				  MAP_operation();
				  //printf("========Destination BLE %d========\r\n", BLE_point);
			  }
			  // Charging State Point
			  else if(!strncmp(comp_buf, slave_addr[3], 6))	// A7EF18 ( Charging Station )
			  {
				  BLE_point = 3;
				  MAP_operation();
				  //printf("========Charge BLE %d========\r\n", BLE_point);
			  }
		  }
	  }
		tail3 = 0;
	 }
	 HAL_UART_Receive_IT(&huart1, &dum3, 1);         // interrupt chain
	}
   // debug
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

int server_con = 0;
int send_fin = 0;

void SEND_INFO()
{
	if((drive_set == 1) && (!send_fin) && (ESP_send != 0))
	{
		switch(BLE_point)
		{
		case 1 :
			send_fin = 1;
			printe("1100");
			break;
		case 2 :
			send_fin = 1;
			printe("1200");
			break;
		case 3 :
			send_fin = 1;
			printe("1300");
			break;
		default :
			break;
		}
	}
	else if((drive_set == 4) && (send_fin))
	{
		send_fin = 0;
		printe("3000");
	}
}

int printe(unsigned char* str) {
   if((HAL_GetTick() - last_printe) < 400) {
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
         //printf("loading..................................\r\n");
         osDelay(500);
         for(i = 0; i < n; i++) {
            HAL_UART_Transmit(&huart6, temp_str + (i * 11), 11, 10);
         }
         HAL_UART_Transmit(&huart6, temp_str + (i * 11), str_len - (i * 11), 10);
      }
      last_printe = HAL_GetTick();
   }
   return 1;
}

// Motor Mode Control
int Mcntr = 0;

//0 : Slow Stop , 1 : Emergency Stop, 2 : Forward, 3: Backward
//4 : Slow Right, 5 : Quick Right
//6 : Slow Left, 7 : Quick Left
void Motor_Mode(int x)
{
	switch(x) {
		  case 0:		// Emergency Stop
			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
			  break;
		  case 1:		// Forward
			  htim1.Instance->CCR1 = 2100;
			  htim1.Instance->CCR3 = 2100;

			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
			  break;
		  case 2:		// Backward
			  htim1.Instance->CCR1 = 2100;
			  htim1.Instance->CCR3 = 2100;
			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
			  break;
		  case 3:		// Slow Right
			  htim1.Instance->CCR1 = 2300;		//ENA
			  htim1.Instance->CCR3 = 2100;		//ENB

			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
			  break;
		  case 4:		// Quick Right
			  htim1.Instance->CCR1 = 2200;
			  htim1.Instance->CCR3 = 2200;

			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
			  break;
		  case 5:		// Slow Left
			  htim1.Instance->CCR1 = 2100;		//ENA
			  htim1.Instance->CCR3 = 2300;		//ENB

			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
			  break;
		  case 6:		// Quick Left
			  htim1.Instance->CCR1 = 2200;
			  htim1.Instance->CCR3 = 2200;

			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
			  break;
	}
}

/* 8E4591 : Start Point, 15DA51 : Corner Point
 37826F : Destination Point, A7EF18 : Charging State Point */
//char* slave_addr[5] = {"8E4591", "15DA51", "37826F", "A7EF18"/*, "9B0C60"*/}; // slave address

int slave_state = 0;
/*point1 (8E4591) : 1
point2 (37826F) : 2
pcharge (A7EF18) : 3*/
//int BLE_point = 0;

// Car Driving complete
int move_comp = 0;

// Drive Mode Set
/*0 : straight, 1 : left, 2 : right, 3 : back*/
//int Move_mode = 0
int straight_param = 0;
int obstacle_state = 0;	// right : + 1, left : -1
int left_param = 0;
int right_param = 0;
int desti_param = 0;

// Ultrasonic
double FF_dist = 0, FR_dist = 0, FL_dist = 0, BB_dist = 0;

/*0 : standby, 1 : Passnger, 2 : Drive, 3 : Watchdog, 4 : Destination, 5 : Charge*/
// drive_set
int RL_cnt = 0;
int ST_cnt = 0;
void driving_mode()
{
	switch(drive_set)
	{
	case 0 :	// Standby
		Motor_Mode(0);	// STOP
		Mcntr = 0;		// Stop
		Move_mode = 0;
		left_param = 0;
		right_param = 0;
		desti_param = 0;
		RL_cnt = 0;
		ST_cnt = 0;
		move_comp = 0;	// initialize
		safe_st = 0;
		slave_state = 0;
		BLE_point = 0;
		ESP_send = 0;
		map_select = 0;
		map_set = 0;
		osDelay(10);
		break;
	case 1 :	// Passenger
		Motor_Mode(0);	// STOP
		Mcntr = 0;		// Stop
		osDelay(10);
		break;
	case 2 :	// Drive
		switch(Move_mode)
		{
		case 0 :	// straight
			switch(ST_cnt)
			{
			case 0 :
				if((FF_dist < 400) && (FL_dist < 400) && (FR_dist < 400))
				{
					//printe("=1=               \r\n");
					Mcntr = 0;
					osDelay(400);
					//Move_mode = 3;	// back
				}
				else if((FF_dist < 600) && (FR_dist > 1000) && (FL_dist > 1000))
				{
					//printe("=2=               \r\n");
					Mcntr = 0;
					osDelay(400);
					ST_cnt = 1;
				}
				else if((FF_dist < 600) && ((FR_dist > 1000) || (FL_dist > 1000)))
				{
					//printe("=3=               \r\n");
					Mcntr = 0;
					osDelay(400);
					ST_cnt = 1;
				}
				if(FF_dist < 400)
				{
					Mcntr = 0;
					osDelay(400);
				}
				else
				{
					// Debug
					//RL_cnt = 0;
					//ST_cnt = 0;
					Mcntr = 1;
					osDelay(10);
				}
				break;
			case 1 :
				if((FF_dist < 600) && (FR_dist > 1000) && (FL_dist > 1000))
				{
					switch(straight_param)
					{
					case 0 :
						if(obstacle_state < 0)
						{
							//printe("=4=               \r\n");
							Mcntr = 4;				// Quick Right
							obstacle_state++;		// right turn 1
							straight_param = 1;
						}
						else
						{
							//printe("=5=               \r\n");
							Mcntr = 6;				// Quick Left
							obstacle_state--;		// left turn 1
							straight_param = 1;
						}
						osDelay(10);
						break;
					case 1 :
						if((MAX_D < -90) || (MAX_D > 90))
						{
							//printe("=6=               \r\n");
							Mcntr = 0;
							osDelay(400);
							straight_param = 0;
							ST_cnt = 0;
						}
						osDelay(10);
						break;
					}
				}
				else if((FF_dist < 600) && ((FR_dist > 1000) || (FL_dist > 1000)))
				{
					switch(straight_param)
					{
					case 0 :
						if(FR_dist > FL_dist)
						{
							//printe("=7=               \r\n");
							Mcntr = 4;				// Quick Right
							obstacle_state++;		// right turn 1
							straight_param = 1;
						}
						else
						{
							//printe("=8=               \r\n");
							Mcntr = 6;				// Quick Left
							obstacle_state--;		// left turn 1
							straight_param = 1;
						}
						osDelay(10);
						break;
					case 1 :
						if((MAX_D < -90) || (MAX_D > 90))
						{
							//printe("=9=               \r\n");
							Mcntr = 0;
							osDelay(400);
							straight_param = 0;
							ST_cnt = 0;
						}
						osDelay(10);
						break;
					}
				}
				break;
			}
			break;
		case 1 :	// left
			if(RL_cnt == 0)
			{
				switch(left_param)
				{
				case 0 :
					//Motor_Mode(1);	// forward
					Mcntr = 1;
					if(FL_dist > 1000)
					{
						Motor_Mode(0);	// STOP
						Mcntr = 0;
						osDelay(400);
						//Read_Z_Angle(&MAX_D);
						left_param = 1;
					}
					osDelay(10);
					break;
				case 1 :
					//Read_Z_Angle(&MAX_D);
					//Motor_Mode(6);	// Quick Left
					Mcntr = 6;
					if(MAX_D < -90)
					{
						Motor_Mode(0);	// STOP
						Mcntr = 0;
						osDelay(400);
						left_param = 2;
					}
					osDelay(10);
					break;
				case 2 :
					//Motor_Mode(1);	// forward
					Mcntr = 1;
					RL_cnt = 1;
					osDelay(100);
					left_param = 0;
					Move_mode = 0;
					break;
				}
			}
			else
			{
			//Motor_Mode(1);	// forward
			Mcntr = 1;
			osDelay(10);
			}
			break;
		case 2 :	// right
			if(RL_cnt == 0)
			{
				switch(right_param)
				{
				case 0 :
					//Motor_Mode(1);	// forward
					Mcntr = 1;
					if(FR_dist > 1000)
					{
						Motor_Mode(0);	// STOP
						Mcntr = 0;
						osDelay(400);
						//Read_Z_Angle(&MAX_D);
						right_param = 1;
					}
					osDelay(10);
					break;
				case 1 :
					//Read_Z_Angle(&MAX_D);
					//Motor_Mode(4);	// Quick right
					Mcntr = 4;
					if(MAX_D > 90)
					{
						Motor_Mode(0);	// STOP
						Mcntr = 0;
						osDelay(400);
						right_param = 2;
					}
					osDelay(10);
					break;
				case 2 :
					//Motor_Mode(1);	// forward
					Mcntr = 1;
					osDelay(100);
					right_param = 0;
					Move_mode = 0;
					break;
				}
			}
			else
			{
				//Motor_Mode(1);	// forward
				Mcntr = 1;
				osDelay(10);
			}
			break;
		case 3 :	// back
			if((FR_dist > 1000) && (FL_dist > 1000))
			{
				Motor_Mode(0);	// STOP
				Mcntr = 0;
				osDelay(400);
				Move_mode = 0;
			}
			else if(BB_dist < 400)
			{
				Motor_Mode(0);	// STOP
				Mcntr = 0;
				osDelay(400);
			}
			else
			{
				//Motor_Mode(2);	// backward
				Mcntr = 2;
				osDelay(10);
			}
			break;
		}

		break;
	case 3 :	// Watchdog
		Motor_Mode(0);	// STOP
		Mcntr = 0;		// Stop
		osDelay(10);
		break;
	case 4 :	//Destination
		switch(desti_param)
		{
		case 0 :
			//Motor_Mode(1);	// forward
			Mcntr = 1;
			if(FF_dist < 700)
			{
				Motor_Mode(0);	// STOP
				Mcntr = 0;
				osDelay(400);
				//Read_Z_Angle(&MAX_D);
				//osDelay(10);
				desti_param = 1;
			}
			osDelay(10);
			break;
		case 1 :
			//Read_Z_Angle(&MAX_D);
			//Motor_Mode(6);	// Quick Left
			Mcntr = 6;
			if(MAX_D < -179)
			{
				Motor_Mode(0);	// STOP
				Mcntr = 0;
				osDelay(400);
				desti_param = 2;
			}
			osDelay(10);
			break;
		case 2 :
			//Motor_Mode(2);	// backward
			Mcntr = 2;
			if(BB_dist < 400)
			{
				Motor_Mode(0);	// STOP
				Mcntr = 0;
				osDelay(400);
				desti_param = 0;
				drive_set = 0;
				move_comp = 1;
			}
			osDelay(10);
			break;
		}
		break;
	case 5 :	// Charging Station
		switch(Move_mode)
		{
		case 0 :	// straight
			switch(ST_cnt)
			{
			case 0 :
				if((FF_dist < 400) && (FL_dist < 400) && (FR_dist < 400))
				{
					//printe("=1=               \r\n");
					Mcntr = 0;
					osDelay(400);
					//Move_mode = 3;	// back
				}
				else if((FF_dist < 600) && (FR_dist > 1000) && (FL_dist > 1000))
				{
					//printe("=2=               \r\n");
					Mcntr = 0;
					osDelay(400);
					ST_cnt = 1;
				}
				else if((FF_dist < 600) && ((FR_dist > 1000) || (FL_dist > 1000)))
				{
					//printe("=3=               \r\n");
					Mcntr = 0;
					osDelay(400);
					ST_cnt = 1;
				}
				if(FF_dist < 400)
				{
					Mcntr = 0;
					osDelay(400);
				}
				else
				{
					ST_cnt = 0;
					Mcntr = 1;
					osDelay(10);
					if((FL_dist < 1000) || (FR_dist < 1000))	RL_cnt = 0;
				}
				break;
			case 1 :
				if((FF_dist < 600) && (FR_dist > 1000) && (FL_dist > 1000))
				{
					switch(straight_param)
					{
					case 0 :
						if(obstacle_state < 0)
						{
							//printe("=4=               \r\n");
							Mcntr = 4;				// Quick Right
							obstacle_state++;		// right turn 1
							straight_param = 1;
						}
						else
						{
							//printe("=5=               \r\n");
							Mcntr = 6;				// Quick Left
							obstacle_state--;		// left turn 1
							straight_param = 1;
						}
						osDelay(10);
						break;
					case 1 :
						if((MAX_D < -90) || (MAX_D > 90))
						{
							//printe("=6=               \r\n");
							Mcntr = 0;
							osDelay(400);
							straight_param = 0;
							ST_cnt = 0;
						}
						osDelay(10);
						break;
					}
				}
				else if((FF_dist < 600) && ((FR_dist > 1000) || (FL_dist > 1000)))
				{
					switch(straight_param)
					{
					case 0 :
						if(FR_dist > FL_dist)
						{
							//printe("=7=               \r\n");
							Mcntr = 4;				// Quick Right
							obstacle_state++;		// right turn 1
							straight_param = 1;
						}
						else
						{
							//printe("=8=               \r\n");
							Mcntr = 6;				// Quick Left
							obstacle_state--;		// left turn 1
							straight_param = 1;
						}
						osDelay(10);
						break;
					case 1 :
						if((MAX_D < -90) || (MAX_D > 90))
						{
							//printe("=9=               \r\n");
							Mcntr = 0;
							osDelay(400);
							straight_param = 0;
							ST_cnt = 0;
						}
						osDelay(10);
						break;
					}
				}
				break;
			}
			break;
		case 1 :	// left
			if(RL_cnt == 0)
			{
				switch(left_param)
				{
				case 0 :
					//Motor_Mode(1);	// forward
					Mcntr = 1;
					if(FL_dist > 1000)
					{
						Motor_Mode(0);	// STOP
						Mcntr = 0;
						osDelay(400);
						//Read_Z_Angle(&MAX_D);
						left_param = 1;
					}
					osDelay(10);
					break;
				case 1 :
					//Read_Z_Angle(&MAX_D);
					//Motor_Mode(6);	// Quick Left
					Mcntr = 6;
					if(MAX_D < -90)
					{
						Motor_Mode(0);	// STOP
						Mcntr = 0;
						osDelay(400);
						left_param = 2;
					}
					osDelay(10);
					break;
				case 2 :
					//Motor_Mode(1);	// forward
					Mcntr = 1;
					RL_cnt = 1;
					osDelay(100);
					left_param = 0;
					Move_mode = 0;
					break;
				}
			}
			else
			{
			//Motor_Mode(1);	// forward
			Mcntr = 1;
			osDelay(10);
			}
			break;
		case 2 :	// right
			if(RL_cnt == 0)
			{
				switch(right_param)
				{
				case 0 :
					//Motor_Mode(1);	// forward
					Mcntr = 1;
					if(FR_dist > 1000)
					{
						Motor_Mode(0);	// STOP
						Mcntr = 0;
						osDelay(400);
						//Read_Z_Angle(&MAX_D);
						right_param = 1;
					}
					osDelay(10);
					break;
				case 1 :
					//Read_Z_Angle(&MAX_D);
					//Motor_Mode(4);	// Quick right
					Mcntr = 4;
					if(MAX_D > 90)
					{
						Motor_Mode(0);	// STOP
						Mcntr = 0;
						osDelay(400);
						right_param = 2;
					}
					osDelay(10);
					break;
				case 2 :
					//Motor_Mode(1);	// forward
					Mcntr = 1;
					osDelay(100);
					right_param = 0;
					Move_mode = 0;
					break;
				}
			}
			else
			{
				//Motor_Mode(1);	// forward
				Mcntr = 1;
				osDelay(10);
			}
			break;
		case 3 :	// back
			if((FR_dist > 1000) && (FL_dist > 1000))
			{
				Motor_Mode(0);	// STOP
				Mcntr = 0;
				osDelay(400);
				Move_mode = 0;
			}
			else if(BB_dist < 400)
			{
				Motor_Mode(0);	// STOP
				Mcntr = 0;
				osDelay(400);
			}
			else
			{
				//Motor_Mode(2);	// backward
				Mcntr = 2;
				osDelay(10);
			}
			break;
		}

		break;
	}
}

void MAP_LOAD()
{
	// in Passenger Mode
	if(drive_set == 1)
	{
		if(!map_set)
		{
			int map_ch = BLE_point + ESP_send;
			switch(map_ch)
			{
			case 5 :
				map_select = 2;
				map_set = 1;
				break;
			case 6 :
				map_select = 4;
				map_set = 1;
				break;
			case 7 :
				map_select = 1;
				map_set = 1;
				break;
			case 9 :
				map_select = 6;
				map_set = 1;
				break;
			case 10 :
				map_select = 3;
				map_set = 1;
				break;
			case 11 :
				map_select = 5;
				map_set = 1;
				break;
			default :
				map_select = 0;
				map_set = 0;
				break;
			}
		}
		else
		{
			if(safe_st)	drive_set = 2;
		}
	}
//	else if(drive_set == 0)
//	{
//		safe_st = 0;
//		slave_state = 0;
//		BLE_point = 0;
//		ESP_send = 0;
//		map_select = 0;
//		map_set = 0;
//	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin) {
	case GPIO_PIN_13:
		//drive_set = 5;		// charge
		//map_select = 4;		// point2 - charge

		// drive_set = 2;	// drive
		//map_select = 1;	// point1 - point2
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

// Voltage Sensor
unsigned int val;			 // Voltage Sensor ADC
unsigned int volt[100];
unsigned int voltage_i = 0;
unsigned int voltage_en = 0;
float voltage = 0.0;         // Current Voltage
float voltage_L = 12.45;		 // Lower Voltage
#define min_voltage 9.00
#define max_voltage 12.45
#define voltage_param (max_voltage - min_voltage)
int volt_cnt = 0;
void Voltage_state()
{
	unsigned int avolt = 0;	     // Average
	volt[voltage_i++] = val;
	//printf("voltage_i : %d \r\n", voltage_i);
	if(voltage_i >= 10) {
		voltage_i = 0;
		voltage_en = 1;
	}
	if(voltage_en) {
		for(int i = 0; i < 10; i++)
		{
			if(volt[i] == 0)	volt[i] = 3100;
		  avolt += volt[i];
		}
		voltage = avolt * 5 * 3.3 / 40950; // 평균 (avolt / 10) * Resolution (3.3 / 4095) * 분배비 5
		//printf("voltage : %.2f \r\n", voltage);
		if(volt_cnt == 0)
		{
			printe("7100");
			volt_cnt++;
		}
		else if(voltage < voltage_L)
		{
		  voltage_L = voltage;
		  float curr_volt = (1 - ((max_voltage - voltage_L) / voltage_param)) * 100;

		  unsigned char vtest_buf1[10];
  		  sprintf(vtest_buf1, "7%03d", (int)curr_volt);
  		  printe(vtest_buf1);

		  if((int)curr_volt < 15)	drive_set = 5;

		  /*debug*/
//		  unsigned char vtest_buf1[50];
//		  sprintf(vtest_buf1, "V:%.2f\r\n", voltage_L);
//		  printe(vtest_buf1);
//		  printf("Average ADC Volt : %.3f \r\n", voltage_L);
//
//		  unsigned char vtest_buf2[50];
//		  sprintf(vtest_buf2, "B:%d\r\n", (int)curr_volt);
//		  printe(vtest_buf2);
		  //printf("배터리 충전 상태 : 7%03d \r\n", (int)curr_volt);
		}
		else if(voltage < 0)
		{
			printe("7000");
		}

//		float curr_volt2 = (1 - ((max_voltage - voltage) / voltage_param)) * 100;
//		printf("배터리 충전 상태 : 7%03d \r\n", (int)curr_volt2);
		voltage_en = 0;
	}
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_I2C1_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Task1 */
  osThreadDef(Task1, StartTask1, osPriorityNormal, 0, 512);
  Task1Handle = osThreadCreate(osThread(Task1), NULL);

  /* definition and creation of Task2 */
  osThreadDef(Task2, StartTask02, osPriorityHigh, 0, 1024);
  Task2Handle = osThreadCreate(osThread(Task2), NULL);

  /* definition and creation of Task03 */
  osThreadDef(Task03, StartTask03, osPriorityIdle, 0, 512);
  Task03Handle = osThreadCreate(osThread(Task03), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);	// Motor PWM1
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);	// Motor PWM2

	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);	// Ultrasonic FF_Echo
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);	// Ultrasonic BB_Echo
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);	// Ultrasonic FR_Echo
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);	// Ultrasonic FL_Echo

	i2c_Gyro_init(&hi2c1);
	Gyro_ModuleSet();
	UART_Start_Receive_IT(&huart1, &dum3, 1);	// Bluetooth
	UART_Start_Receive_IT(&huart2, &dum2, 1);	// Putty
	UART_Start_Receive_IT(&huart6, &dum1, 1);	// ESP8266
	HAL_TIM_Base_Start_IT(&htim5);				// bluetooth AT Command Interrupt
	HAL_ADC_Start_DMA(&hadc1, &val, 1);
	HAL_TIM_Base_Start_IT(&htim3);				// Voltage_state Operation

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim1.Init.Prescaler = 16-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 7500-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 4200-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 10000-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, BBtrig_Pin|FRtrig_Pin|FLtrig_Pin|FFtrig_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : BBtrig_Pin FRtrig_Pin FLtrig_Pin FFtrig_Pin */
  GPIO_InitStruct.Pin = BBtrig_Pin|FRtrig_Pin|FLtrig_Pin|FFtrig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_Pin IN4_Pin IN2_Pin IN3_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN4_Pin|IN2_Pin|IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint32_t lastMotorMode = 0;

/* debug */
uint32_t lastPrintTime = 0;
uint32_t currentTime;

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask1 */
/**
  * @brief  Function implementing the Task1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask1 */
void StartTask1(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  // Motor Mode Change only detect Mcntr change
	  if(lastMotorMode != Mcntr)
	  {
		lastMotorMode = Mcntr;
		Motor_Mode(lastMotorMode);
	  }
	  driving_mode();
//	  unsigned char testbuf2[50];
//	  sprintf(testbuf2[50],"D:%d,B:%d,W:%d,S:%d,M:%d\r\n", drive_set, BLE_point, ESP_send, safe_st, map_set);
//	  printe(testbuf2);
	  printf("D:%d,B:%d,W:%d,S:%d,M:%d\r\n", drive_set, BLE_point, ESP_send, safe_st, map_set);

	  osDelay(100);
//	  unsigned char testbuf2[50];
//	  sprintf(testbuf2,"z:%d,dp:%d,ds:%d,Mm:%d                     \r\n",MAX_D, desti_param, drive_set, Move_mode);
//	  printe(testbuf2);

	  /* Debug */
	  //printf("%d\r\n",Mcntr);
//	  unsigned char testbuf2[50];
//	  sprintf(testbuf2,"F:%d,R:%d,L:%d,B:%d                   \r\n",(int)FF_dist, (int)FR_dist, (int)FL_dist, (int)BB_dist);
//	  printe(testbuf2);
	  osDelay(10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
	// 1. FFtrig
	HAL_GPIO_WritePin(FFtrig_GPIO_Port, FFtrig_Pin, GPIO_PIN_SET);  // FFtrig HIGH
	osDelay(1);  // 1ms High
	HAL_GPIO_WritePin(FFtrig_GPIO_Port, FFtrig_Pin, GPIO_PIN_RESET);  // FFtrig LOW
	//driving_mode();
	osDelay(100);  // Calculation Delay

	// 2. FRtrig
	HAL_GPIO_WritePin(FRtrig_GPIO_Port, FRtrig_Pin, GPIO_PIN_SET);  // FFtrig HIGH
	osDelay(1);  // 1ms High
	HAL_GPIO_WritePin(FRtrig_GPIO_Port, FRtrig_Pin, GPIO_PIN_RESET);  // FFtrig LOW
	//driving_mode();
	osDelay(100);  // Calculation Delay

	// 3. FLtrig
	HAL_GPIO_WritePin(FLtrig_GPIO_Port, FLtrig_Pin, GPIO_PIN_SET);  // FFtrig HIGH
	osDelay(1);  // 1ms High
	HAL_GPIO_WritePin(FLtrig_GPIO_Port, FLtrig_Pin, GPIO_PIN_RESET);  // FFtrig LOW
	//driving_mode();
	osDelay(100);  // Calculation Delay

	// 4. BBtrig
	HAL_GPIO_WritePin(BBtrig_GPIO_Port, BBtrig_Pin, GPIO_PIN_SET);  // FFtrig HIGH
	osDelay(1);  // 1ms High
	HAL_GPIO_WritePin(BBtrig_GPIO_Port, BBtrig_Pin, GPIO_PIN_RESET);  // FFtrig LOW
	//driving_mode();
	osDelay(100);  // Calculation Delay
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the Task03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
//	Read_Z_Angle(&MAX_D);
//	osDelay(1000);
	  // ESP Set
	//Voltage_state();
	if(esp_set == 0)
	{
		HAL_UART_Transmit(&huart6, "AT+CIPMUX=1", 11, 10);            // 0: single connection mode, 1: multi connection mode
		HAL_UART_Transmit(&huart6, "\r\n", 2, 10);
		osDelay(3000);
	}
	else if(TCP_connect && (esp_set == 1))
	{
		//HARMAN TCP SERVER CONNECT
		HAL_UART_Transmit(&huart6, "AT+CIPSTART", 11, 10);
		HAL_UART_Transmit(&huart6, "=4,\"TCP\",\"1", 11, 10);
		HAL_UART_Transmit(&huart6, "92.168.0.6", 10, 10);
		HAL_UART_Transmit(&huart6, "9\",3000\r\n", 9, 10);      // SERVER Connect
		osDelay(2000);
	}
	else if(drive_set == 0)
	{
		if(server_con == 0)
		{
			printe("8000");
			server_con = 1;
			osDelay(10);
		}
		Voltage_state();
		osDelay(1000);
	}
	else if(drive_set == 1)
	{
		SEND_INFO();
		MAP_LOAD();
		osDelay(100);
	}
	else
	{
		SEND_INFO();
		Read_Z_Angle(&MAX_D);
		osDelay(10);
	}
	osDelay(1);
  }
  /* USER CODE END StartTask03 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM5) {
			// Timer5 interrupt : 0.5 sec Period
		if(drive_set != 0) HAL_UART_Transmit(&huart1, "AT+INQ\r\n", 8, 10);
		//else				Voltage_state();
		}
//	else if (htim->Instance == TIM3) {
//			// Timer3 interrupt	0.06 sec Period
//		if(drive_set == 0)	Voltage_state();
//	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
