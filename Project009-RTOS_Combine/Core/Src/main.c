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
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART1_UART_Init(void);
void StartTask1(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

/* ESP8266 Send
point1 (8E4591) : 6
point2 (37826F) : 4
pcharage (A7EF18) : 2 */
int ESP_send = 0;

// Driving Mode Select
int drive_set = 0;

/* 8E4591 : Start Point, 15DA51 : Corner Point
 37826F : Destination Point, A7EF18 : Charging State Point */
char* slave_addr[5] = {"8E4591", "15DA51", "37826F", "A7EF18"/*, "9B0C60"*/}; // slave address
int BLE_point = 0;
/*point1 (8E4591) : 1
point2 (37826F) : 2
pcharge (A7EF18) : 3*/
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
           if(!strncmp(buf1, "WIFI GOT IP", 11)) {
			WIFI_connect = 0;
		 }
           if(!strncmp(buf1, "WIFI DISCONNECT", 15)) {
        	   WIFI_connect = 1;
           }
           if(!strncmp(buf1, "OK", 2)) {
        	   esp_set++;
           }
           if(!strncmp(buf1, "+IPD,", 5)) {
			 switch(buf1[9]) {
				case '0':	// Emergency & Normal State Set
				   //buf1[10] 0 : emergency, 1 : Force OK, 2 : Heart Rate OK, 3 : Total OK
					unsigned char ESP_words0 = buf1[10];
					if(drive_set == 0)		// Standby Mode
					{
						if(ESP_words0 == '1')	drive_set = 1;
						else					drive_set = 0;
					}
					else if(drive_set == 1)	// Passenger Mode
					{
						if(ESP_words0 == '3')	drive_set = 2;
						else					drive_set = 1;
					}
					else if(drive_set == 2)	// Driving Mode
					{
						if(ESP_words0 == '0')	drive_set = 3;
						else					drive_set = 2;
					}
					else if(drive_set == 3)	// Watchdog Mode
					{
						if(ESP_words0 == '3')	drive_set = 2;
						else					drive_set = 3;
					}
					break;
//				case '1':	// Origin is xxx (Not use in Server)
//				   // Origin is xxx
//				   break;
				case '2':	// Destination Set
					unsigned char ESP_words1 = buf1[10];
					if(drive_set == 1)
					{
						if(ESP_words0 == '6')		ESP_send = 6;
						else if(ESP_words0 == '4')	ESP_send = 4;
						else if(ESP_words0 == '2')	ESP_send = 2;
					}
				   break;
//				case '3':	// Arrive at Destination (Not use in Server)
//				   break;
//				case '4':	// Hall Sensor
//				   break;
//				case '5':	// Force Sensor
//				   break;
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
			  if(!strncmp(comp_buf, slave_addr[0], 6))	// 8E4591
			  {
				  BLE_point = 1;
			  }
			  // Corner Point
			  else if(!strncmp(comp_buf, slave_addr[1], 6))	// 15DA51
			  {
				  BLE_point = 4;
			  }
			  // Destination Point
			  else if(!strncmp(comp_buf, slave_addr[2], 6))	// 37826F
			  {
				  BLE_point = 2;
			  }
			  // Charging State Point
			  else if(!strncmp(comp_buf, slave_addr[3], 6))	// A7EF18
			  {
				  BLE_point = 3;
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
			  htim1.Instance->CCR1 = 1100;
			  htim1.Instance->CCR3 = 1100;

			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
			  break;
		  case 2:		// Backward
			  htim1.Instance->CCR1 = 1100;
			  htim1.Instance->CCR3 = 1100;
			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
			  break;
		  case 3:		// Slow Right
			  htim1.Instance->CCR1 = 1300;		//ENA
			  htim1.Instance->CCR3 = 1100;		//ENB

			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
			  break;
		  case 4:		// Quick Right
			  htim1.Instance->CCR1 = 1200;
			  htim1.Instance->CCR3 = 1200;

			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
			  break;
		  case 5:		// Slow Left
			  htim1.Instance->CCR1 = 1100;		//ENA
			  htim1.Instance->CCR3 = 1300;		//ENB

			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
			  break;
		  case 6:		// Quick Left
			  htim1.Instance->CCR1 = 1200;
			  htim1.Instance->CCR3 = 1200;

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

/* Map List
map1 : point1 - corner1 - point2 = 5
map2 : point1 - corner1 - pcharge = 3
map3 : point2 - corner1 - point1 = 8
map4 : point2 - corner1 - pcharge = 4
map5 : pcharge - corner1 - point1 = 9
map6 : pcharge - corner1 - point2 = 7*/
int map_select = 0;
int map_set = 0;

// Car Driving complete
int move_comp = 0;

// Drive Mode Set
int Move_mode = 0;

void driving_mode()
{
	switch(drive_set)
	{
	case 0 :	// Standby
		move_comp = 0;
		break;
	case 1 :	// Passenger

		break;
	case 2 :	// Drive

		break;
	case 3 :	// Watchdog
		break;
	case 4 :	//Destination
		move_comp = 1;
		break;
	case 5 :	// Charging Station
		break;
	}
}

void MAP_LOAD()
{
	// in Passenger Mode
	if(drive_set == 1)
	{
		if(BLE_point > 0)		slave_state = 1;
		else if(slave_state && (!map_set))
		{
			int map_ch = BLE_point + ESP_send;
			switch(map_ch)
			{
			case 3 :
				map_select = 2;
				map_set = 1;
				break;
			case 4 :
				map_select = 4;
				map_set = 1;
				break;
			case 5 :
				map_select = 1;
				map_set = 1;
				break;
			case 7 :
				map_select = 6;
				map_set = 1;
				break;
			case 8 :
				map_select = 3;
				map_set = 1;
				break;
			case 9 :
				map_select = 5;
				map_set = 1;
				break;
			default :
				map_select = 0;
				break;
			}
		}
		else if(map_set)
		{
			if(move_comp)
			{
				// initialize
				slave_state = 0;
				BLE_point = 0;
				drive_set = 0;
				ESP_send = 0;
				map_set = 0;
			}
			else			map_set = 1;
		}
	}
}

int btn = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin) {
	case GPIO_PIN_13:

		btn = 1;
		drive_set = 1;
//		if(Mcntr > 6)	Mcntr = 0;
//		else			Mcntr++;
		break;
  }
}

double FF_dist = 0, FR_dist = 0, FL_dist = 0, BB_dist = 0;
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

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if (htim->Instance == TIM5) {
//		// Timer5 interrupt : 1 sec Period
//		HAL_UART_Transmit(&huart1, "AT+INQ\r\n", 8, 10);
//	}
////	else if (htim->Instance == TIM3) {
////	        // Timer3 interrupt	0.06 sec Period
////		Voltage_state();
////	}
//}

// Gyro Max Value
double MAX_D = 0;

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
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_I2C1_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
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
  osThreadDef(Task2, StartTask02, osPriorityHigh, 0, 512);
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
  htim1.Init.Prescaler = 21-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4000-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  htim5.Init.Prescaler = 8400-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
	  // Move Mode about Ultrasonic
//	  if((FF_dist < 100) && (FR_dist < 100) && (FL_dist < 100))
//	  {
//		  Mcntr = 2;
//	  }
//	  else if((FF_dist < 100) && (FR_dist < 100))
//	  {
//		  Mcntr = 6;
//	  }
//	  else if((FF_dist < 100) && (FL_dist < 100))
//	  {
//		  Mcntr = 4;
//	  }
//	  else if(FF_dist < 100)
//	  {
//		  Mcntr = 0;
//	  }
//	  else if(FR_dist < 100)
//	  {
//		  Mcntr = 5;
//	  }
//	  else if(FL_dist < 100)
//	  {
//		  Mcntr = 3;
//	  }
//	  else
//	  {
//		  Mcntr = 1;
//	  }

	  /* Debug */
	  //printf("%d\r\n",Mcntr);
	  unsigned char testbuf2[50];
	  sprintf(testbuf2,"F:%d,R:%d,L:%d,B:%d                   \r\n",(int)FF_dist, (int)FR_dist, (int)FL_dist, (int)BB_dist);
	  printe(testbuf2);
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
	if(FF_dist < 400)
	{
		Mcntr = 0;
		Motor_Mode(0);	// STOP
		osDelay(10);
	}
	else if(FL_dist > 1000)
	{
		Motor_Mode(0);
		osDelay(500);
		while(MAX_D > -90)
		{
			//printf("====%.2f====\r\n", MAX_D);
			Read_Z_Angle(&MAX_D);
			Mcntr = 6;
			//Motor_Mode(6);	// Quick Left
			osDelay(10);
		}
		Mcntr = 1;
		osDelay(800);
	}
	else
	{
		if(btn)
		{
			Mcntr = 1;
		}
	}
	osDelay(100);  // Calculation Delay

	// 2. FRtrig
	HAL_GPIO_WritePin(FRtrig_GPIO_Port, FRtrig_Pin, GPIO_PIN_SET);  // FFtrig HIGH
	osDelay(1);  // 1ms High
	HAL_GPIO_WritePin(FRtrig_GPIO_Port, FRtrig_Pin, GPIO_PIN_RESET);  // FFtrig LOW
	osDelay(100);  // Calculation Delay

	// 3. FLtrig
	HAL_GPIO_WritePin(FLtrig_GPIO_Port, FLtrig_Pin, GPIO_PIN_SET);  // FFtrig HIGH
	osDelay(1);  // 1ms High
	HAL_GPIO_WritePin(FLtrig_GPIO_Port, FLtrig_Pin, GPIO_PIN_RESET);  // FFtrig LOW
	osDelay(100);  // Calculation Delay

	// 4. BBtrig
	HAL_GPIO_WritePin(BBtrig_GPIO_Port, BBtrig_Pin, GPIO_PIN_SET);  // FFtrig HIGH
	osDelay(1);  // 1ms High
	HAL_GPIO_WritePin(BBtrig_GPIO_Port, BBtrig_Pin, GPIO_PIN_RESET);  // FFtrig LOW
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
	  // ESP Set
	if(!esp_set)
	{
		HAL_UART_Transmit(&huart6, "AT+CWMODE=1", 11, 10);            // 1: CLIENT MODE, 2: SERVER MODE, 3: Multi mode
		HAL_UART_Transmit(&huart6, "\r\n", 2, 10);
		osDelay(10);
		osDelay(3000);
	}
	else if(WIFI_connect && (esp_set == 1))
	{
		HAL_UART_Transmit(&huart6, "AT+CWJAP=\"P", 11, 10);
		HAL_UART_Transmit(&huart6, "rocessor2.4", 11, 10);
		HAL_UART_Transmit(&huart6, "G\",\"Process", 11, 10);
		HAL_UART_Transmit(&huart6, "or1234\"\r\n", 9, 10);      // SERVER Connect
		osDelay(2000);
	}
	else if(esp_set == 2)
	{
		HAL_UART_Transmit(&huart6, "AT+CIPMUX=1", 11, 10);            // 0: single connection mode, 1: multi connection mode
		HAL_UART_Transmit(&huart6, "\r\n", 2, 10);
		osDelay(3000);
	}
	else if(TCP_connect && (esp_set == 3))
	{
		//HARMAN TCP SERVER CONNECT
		HAL_UART_Transmit(&huart6, "AT+CIPSTART", 11, 10);
		HAL_UART_Transmit(&huart6, "=4,\"TCP\",\"1", 11, 10);
		HAL_UART_Transmit(&huart6, "92.168.0.65", 11, 10);
		HAL_UART_Transmit(&huart6, "\",3000\r\n", 8, 10);      // SERVER Connect
		osDelay(2000);
	}
	osDelay(100);
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
			// Timer5 interrupt : 1 sec Period
			HAL_UART_Transmit(&huart1, "AT+INQ\r\n", 8, 10);
		}
	//	else if (htim->Instance == TIM3) {
	//	        // Timer3 interrupt	0.06 sec Period
	//		Voltage_state();
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
