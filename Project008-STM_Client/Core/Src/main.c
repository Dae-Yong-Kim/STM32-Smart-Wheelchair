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
#include <string.h>
#include "heartRate.h"
#include "lcd_driver.h"
#include "lcd_gui.h"
#include "lcd_touch.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
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

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define RATE_SIZE 10
#define BUF_SIZE 100

uint16_t W_addr = 0xAE;
uint16_t R_addr = 0xAF;
uint8_t data;
char buf1[BUF_SIZE], buf2[BUF_SIZE]; // DMA Buffer
char dum1, dum2;
int head1 = 0, head2 = 0, tail1 = 0, tail2 = 0;
int belt_mode = 0, sit_mode = 0;	// belt mode(0: unbuckle, 1: buckle), sit mode(0: stand, 1: sit)
int belt_pe = 1, sit_pe = 1;		// belt print enable, sit print enable
int TCP_connect = 1, WIFI_connect = 1;
int last_printe = 0;
int pre_battery = 100, battery = 100, lcd_set = 0, tp_en = 0;
int hall_set = 0, force_set = 0, wifi_set = 0;
int beatAvg = 0, main_sc = 0, emer_sc = 0;
int current_screen = 0; //0: main, 1: emergency, 2: A, 3: B
// SM has to change huart1 to huart6
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
			HAL_UART_Transmit(&huart1, "AT+CIPSEND=", 11, 10);
			sprintf(temp_AT, "4,%d\r\n", str_len);
			if(str_len < 10) {
				HAL_UART_Transmit(&huart1, temp_AT, 5, 10);
			}
			else {
				HAL_UART_Transmit(&huart1, temp_AT, 6, 10);
			}
			int n = str_len / 11;
			int i = 0;
			printf("loading..................................\r\n");
			for(i = 0; i < n; i++) {
				HAL_UART_Transmit(&huart1, temp_str + (i * 11), 11, 10);
			}
			HAL_UART_Transmit(&huart1, temp_str + (i * 11), str_len - (i * 11), 10);
		}
		last_printe = HAL_GetTick();
	}
	return 1;
}

void i2c_scan()
{
   for(int addr=0; addr<256; addr++)
   {
      if(HAL_I2C_IsDeviceReady(&hi2c1, addr, 1, 10 /* ms*/) == HAL_OK)
      {
         printf("  %02x ", addr);
      }
      else
      {
         printf("  .  ");
      }
      if((addr % 16) == 15)   printf("\r\n");
   }
}

void Moduleset()
{
    // Reset
    data = 0x40;
    HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x09, 1, &data, 1, 1000);
    while(data & 0x40) {
    	HAL_I2C_Mem_Read(&hi2c1, R_addr, 0x09, 1, &data, 1, 1000);
    }
    HAL_Delay(10);

    // Interrupt Clear
    HAL_I2C_Mem_Read(&hi2c1, R_addr, 0x00, 1, &data, 1, 1000);
    HAL_I2C_Mem_Read(&hi2c1, R_addr, 0x01, 1, &data, 1, 1000);

    // Interrupt Enable
    data = 0x40;  //
    HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x02, 1, &data, 1, 1000);
    HAL_Delay(10);

    // FIFO Pointer
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x04, 1, &data, 1, 1000);
    HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x05, 1, &data, 1, 1000);
    HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x06, 1, &data, 1, 1000);

    // FIFO Set
    data = 0x50;  //0b0101_0000 4 Sample Average: 4, RollOver Enable, Full Data Interrupt
    HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x08, 1, &data, 1, 1000);
    HAL_Delay(10);

    // Mode Set
    data = 0x02;  // 0x01 : IR, 0x02 : RED, 0x03 : Multi
    HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x09, 1, &data, 1, 1000);
    HAL_Delay(10);

    // Sampling Set - Sampling Rate : 50Hz, Pulse Width : 411μs
    data = 0x2F;  // 0b00101111
    HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x0A, 1, &data, 1, 1000);
    HAL_Delay(10);

    // LED Current set
    data = 0x1F;  // MAX 50mA (0xFF) // 0x20: 14.2mA
    HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x0C, 1, &data, 1, 1000);
    HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x0D, 1, &data, 1, 1000);
    HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x10, 1, &data, 1, 1000);
    HAL_Delay(10);

    // Slot Set
	data = 0x01;  // Slot1 : RED
	HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x11, 1, &data, 1, 1000);
	HAL_Delay(10);

    // Part ID= 0x15
    HAL_I2C_Mem_Read(&hi2c1, W_addr, 0xFF, 1, &data, 1, 1000);
    printf("Part ID: 0x%02X\r\n", data);  // Default 0x15
}

int getIR() {
	uint8_t temp_dataBuffer[3];

	HAL_I2C_Mem_Read(&hi2c1, R_addr, 0x07, 1, temp_dataBuffer, 3, 1000);

	return (((uint32_t)temp_dataBuffer[0] << 16) | ((uint32_t)temp_dataBuffer[1] << 8) | (uint32_t)temp_dataBuffer[2]) & 0x03FFFF;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//printf("1111111\r\n");
  switch(GPIO_Pin) {
	  case HALL_Pin:
		  if(hall_set) {
			  if(HAL_GPIO_ReadPin(HALL_GPIO_Port, HALL_Pin)) {
				  if(belt_mode) {
					  printf("unbuckle seat belt\r\n");
					  belt_mode = 0;
				  }
			  }
			  else {
				  if(!belt_mode) {
					  printf("buckle seat belt\r\n");
					  belt_mode = 1;
				  }
			  }
			  belt_pe = 1;
		  }
		  break;
	  case FORCE_Pin:
		  if(force_set) {
			  if(HAL_GPIO_ReadPin(FORCE_GPIO_Port, FORCE_Pin)) {
				  if(sit_mode) {
					  printf("stand\r\n");
					  sit_mode = 0;
				  }
			  }
			  else {
				  if(!sit_mode) {
					  printf("sit\r\n");
					  sit_mode = 1;
				  }
			  }
			  sit_pe = 1;
		  }

		  break;
	  case TP_IRQ_Pin:
		  if(lcd_set) {
			  tp_en = 1;
		  }

		  break;
	  case B1_Pin:
		  if(wifi_set) {
			  ESP8266_client_init();
		  }
		  break;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   if(huart == &huart1)
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

    		  //printf("%s\r\n", buf1);

    		  char comp_buf[BUF_SIZE];
    		  sprintf(comp_buf, "%s", buf1);
    		  if(!strncmp(comp_buf, "4,CONNECT", 9) || !strncmp(comp_buf, "ALREADY CONNECTED", 17)) {
    			  TCP_connect = 0;
    		  }
    		  if(!strncmp(comp_buf, "4,CLOSED", 8)) {
    			  TCP_connect = 1;
    		  }
    		  if(!strncmp(buf1, "WIFI GOT IP", 11)) {
    			  WIFI_connect = 0;
			  }
			  if(!strncmp(buf1, "WIFI DISCONNECT", 15)) {
				  WIFI_connect = 1;
			  }


			  if(!strncmp(comp_buf, "+IPD,", 5)) {
				  switch(comp_buf[9]) {													// Recieve MSG From Client
					  case '0':														// Emergency (Not use in Server)
						  if(comp_buf[10] = '0') {
							  emer_sc = 1;
							  /*LCD_Clear(BACKGROUND_COLOR, beatAvg, battery);  // 화면 클리어
							  EmergencyMessage();*/  // 긴급 메시지 표시 함수 호출
						  }
						  else {
							  main_sc = 1;
							  /*LCD_Clear(BACKGROUND_COLOR, beatAvg, battery);
							  Load_Touch_Draw();*/
						  }
						  break;
					  case '1':														// Origin is xxx (Not use in Server)
						  // Origin is xxx
						  break;
					  case '2':														// Destination is xxx
						  // recieve 2xxx value ==> Destination is xxx
						  break;
					  case '3':														// Arrive at Destination (Not use in Server)
						  main_sc = 1;
						  /*LCD_Clear(BACKGROUND_COLOR, beatAvg, battery);
						  Load_Touch_Draw();*/
						  break;
					  case '4':														// Hall Sensor
						  break;
					  case '5':														// Force Sensor
						  break;
					  case '6':														// Heart Beat Sensor
						  break;
					  case '7':														// Voltage Sensor (Not use in Server)
						  printf("remain battery: %s%", comp_buf + 10);
						  battery = atoi(comp_buf + 10);
						  break;
					  case '8':														// STM Client
						  break;
				  }
			  }
		  }
    	  tail1 = 0;
      }
      HAL_UART_Receive_IT(&huart1, &dum1, 1);         // interrupt chain
   }

   else if(huart == &huart2)
   {
      buf2[tail2++] = dum2;
      HAL_UART_Transmit(&huart2, &dum2, 1, 10); // terminal echo
      if(dum2 == '\r')  // CR : 0x0d
      {
         HAL_UART_Transmit(&huart2, "\n", 1, 10); // terminal echo

         buf2[tail2++] = '\n'; // == HAL_UART_Transmit(&huart1, "\n", 1, 10);
         HAL_UART_Transmit(&huart1, buf2, tail2, 10);   // AT Command
//         HAL_UART_Transmit(&huart1, "\n", 1, 10);
         tail2 = 0;
      }
      HAL_UART_Receive_IT(&huart2, &dum2, 1);
   }
}

void ESP8266_client_init(){
	/*HAL_UART_Transmit(&huart1, "AT+RST\r\n", 8, 10);					// RESET
	HAL_Delay(1000);*/

	HAL_UART_Transmit(&huart1, "AT+CWMODE=1", 11, 10);				// 1: CLIENT MODE, 2: SERVER MODE, 3: Multi mode
	HAL_UART_Transmit(&huart1, "\r\n", 2, 10);
	HAL_Delay(10);

	while(WIFI_connect) { // HARMAN WIFI CONNECT
		HAL_UART_Transmit(&huart1, "AT+CWJAP=\"P", 11, 10);
		HAL_UART_Transmit(&huart1, "rocessor2.4", 11, 10);
		HAL_UART_Transmit(&huart1, "G\",\"Process", 11, 10);
		HAL_UART_Transmit(&huart1, "or1234\"\r\n", 9, 10);      // SERVER Connect
		HAL_Delay(1000);
	}

	/*while(WIFI_connect) { // DY WIFI CONNECT
		HAL_UART_Transmit(&huart1, "AT+CWJAP=\"", 10, 10);
		HAL_UART_Transmit(&huart1, "ssosso\",\"19", 11, 10);
		HAL_UART_Transmit(&huart1, "980501\"\r\n", 9, 10);			// SERVER Connect
		HAL_Delay(1000);
	}*/

	printf("connect WIFI\r\n");

	HAL_UART_Transmit(&huart1, "AT+CIPMUX=1", 11, 10);				// 0: single connection mode, 1: multi connection mode
	HAL_UART_Transmit(&huart1, "\r\n", 2, 10);
	HAL_Delay(10);

	// STM TCP SERVER CONNECT
	/*while(TCP_connect) {
		HAL_UART_Transmit(&huart1, "AT+CIPSTART", 11, 10);
		HAL_UART_Transmit(&huart1, "=4,\"TCP\",\"1", 11, 10);
		HAL_UART_Transmit(&huart1, "92.168.4.1\"", 11, 10);
		HAL_UART_Transmit(&huart1, ",3000\r\n", 7, 10);		// SERVER Connect
		HAL_Delay(1000);
	}*/

	// DY TCP SERVER CONNECT
	/*while(TCP_connect) {
		HAL_UART_Transmit(&huart1, "AT+CIPSTART", 11, 10);
		HAL_UART_Transmit(&huart1, "=4,\"TCP\",\"1", 11, 10);
		HAL_UART_Transmit(&huart1, "92.168.96.1", 11, 10);
		HAL_UART_Transmit(&huart1, "60\",3000\r\n", 10, 10);		// SERVER Connect
		HAL_Delay(1000);
	}*/

	// HARMAN TCP SERVER CONNECT
	while(TCP_connect) {
		HAL_UART_Transmit(&huart1, "AT+CIPSTART", 11, 10);
		HAL_UART_Transmit(&huart1, "=4,\"TCP\",\"1", 11, 10);
		HAL_UART_Transmit(&huart1, "92.168.0.69", 11, 10);
		HAL_UART_Transmit(&huart1, "\",3000\r\n", 8, 10);      // SERVER Connect
		HAL_Delay(1000);
	}


	while(!printe("8100"));
	HAL_Delay(100);
	printf("connect TCP server\r\n");

	/*HAL_UART_Transmit(&huart1, "AT+CIFSR", 11, 10);					// Show IP & MAC Address
	HAL_UART_Transmit(&huart1, "\r\n", 2, 10);
	HAL_Delay(10);*/
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  ProgramStart("STM Client");
  UART_Start_Receive_IT(&huart1, &dum1, 1);
  UART_Start_Receive_IT(&huart2, &dum2, 1);
  ESP8266_client_init();
  i2c_scan();
  Moduleset();
  LCD_Clear(BACKGROUND_COLOR, 0, 100);
  TP_Init();
  Lcd_Init();

  Load_Touch_Draw();

  // LED Current set
  data = 0x0A;  // MAX 50mA (0xFF) // 0x20: 14.2mA
  HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x0C, 1, &data, 1, 1000);
  lcd_set = 1;
  force_set = 1;
  hall_set = 1;
  wifi_set = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int rates[RATE_SIZE] = {0, };
  int rateSpot = 0;
  double beatsPerMinute = 0;
  int beat_cnt = 0;
  int lastBeat = 0;
  int pre_beat_mode = 3, beat_mode = 0;	//0: no finger, 1: normal, 2: abnormal
  int IRValue = 0;
  int last_bpm_update_time = 0;
  int pre_bpm = 0;
  while (1)
  {
	  HAL_I2C_Mem_Read(&hi2c1, R_addr, 0x00, 1, &data, 1, 1000);
	  if(data & 0x40) {
		  IRValue = getIR();

		  if(checkForBeat(IRValue)) {
			  int delta = HAL_GetTick() - lastBeat;
			  lastBeat = HAL_GetTick();

			  beatsPerMinute = 60 / (delta / 1000.0);

			  if ((beatsPerMinute < 255) && (beatsPerMinute > 20)) {
				rates[rateSpot++] = beatsPerMinute; //Store this reading in the array
				rateSpot %= RATE_SIZE; //Wrap variable

				//Take average of readings
				for (int x = 0 ; x < RATE_SIZE ; x++){
					beatAvg += rates[x];
				}
				beatAvg /= RATE_SIZE;
			  }

			  if(pre_beat_mode != beat_mode) {
				  beat_cnt++;
			  }
			  else {
				  beat_cnt = 0;
			  }
		  }

		  if(IRValue < 10000) {
			  beat_mode = 0;
		  }
		  else {
			  if((beatsPerMinute > 50) && (beatsPerMinute < 120)) {
				  beat_mode = 1;
			  }
			  else {
				  beat_mode = 2;
			  }
			  printf("IRValue: %5d, BPM: %5.2f, avg_BPM: %3d\r\n", IRValue, beatsPerMinute, beatAvg);
		  }
	  }

	  if (HAL_GetTick() - last_bpm_update_time >= 1000)  // 1초�? 경과?��?�� ?��
	  {
		  //Update_BPM_Text(38, 23, beatAvg);
		  //heartbeat = rand() % 41 + 60;  // 60 ~ 100 ?��?��?�� ?��?�� BPM �?? ?��?��
		  if((IRValue >= 10000) && (pre_bpm != beatAvg)) {
			  Update_BPM_Text(38, 23, beatAvg);
			  pre_bpm = beatAvg;
		  }
		  last_bpm_update_time = HAL_GetTick();  // 마�?�?? ?��?��?��?�� ?���?? 기록
	  }

	  // beat printe
	  if((pre_beat_mode != beat_mode) && ((beat_cnt == 10) || (beat_mode == 0))) {
		  switch(beat_mode) {
			  case 0:
				  while(!printe("6000"));
				  Update_BPM_Text(38, 23, 0);  // BPM ?��?��?�� 갱신
				  break;
			  case 1:
				  while(!printe("6100"));
				  Update_BPM_Text(38, 23, beatAvg);
				  break;
			  case 2:
				  while(!printe("6200"));
				  Update_BPM_Text(38, 23, beatAvg);
				  break;
		  }
		  beat_cnt = 0;
		  pre_beat_mode = beat_mode;
	  }

	  // sit & belt printe
	  if(belt_pe) {
		  if(belt_mode) {
			  while(!printe("4100"));
		  }
		  else {
			  while(!printe("4000"));
		  }
		  belt_pe = 0;
	  }
	  if(sit_pe) {
		  if(sit_mode) {
			  while(!printe("5100"));
		  }
		  else {
			  while(!printe("5000"));
		  }
		  sit_pe = 0;
	  }

	  if(tp_en) {
		  if(IRValue < 10000) {
			  TP_test(0, battery, &current_screen);
		  }
		  else {
			  TP_test(beatAvg, battery, &current_screen);
		  }
		  tp_en = 0;
	  }

	  if(pre_battery != battery) {
		  Draw_Battery(battery);
		  pre_battery = battery;
	  }

	  if(main_sc && (current_screen != 0)) {
		  LCD_Clear(BACKGROUND_COLOR, beatAvg, battery);
		  Load_Touch_Draw();
		  current_screen = 0;
	  }

	  if(emer_sc && (current_screen != 1)) {
		  LCD_Clear(BACKGROUND_COLOR, beatAvg, battery);  // 화면 클리어
		  EmergencyMessage();
		  current_screen = 1;
	  }

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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 640-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  HAL_GPIO_WritePin(GPIOB, SD_CS_Pin|TP_CS_Pin|LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RST_Pin|LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_CS_Pin TP_CS_Pin LCD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin|TP_CS_Pin|LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : HALL_Pin FORCE_Pin */
  GPIO_InitStruct.Pin = HALL_Pin|FORCE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RST_Pin LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin|LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TP_BUSY_Pin */
  GPIO_InitStruct.Pin = TP_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TP_BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TP_IRQ_Pin */
  GPIO_InitStruct.Pin = TP_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TP_IRQ_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
