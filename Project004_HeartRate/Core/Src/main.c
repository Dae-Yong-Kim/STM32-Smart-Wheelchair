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
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#define RATE_SIZE 20
//#include "stm32fxxx_hal.h"
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

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t W_addr = 0xAE;
uint16_t R_addr = 0xAF;
uint8_t data;

/*void Moduleset()
{
    // Reset
    data = 0x40;
    HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x09, 1, &data, 1, 1000);
    HAL_Delay(100);

    // FIFO Set
    data = 0xD0;  // 4 Sample Average: 32, RollOver Enable, Full Data Interrupt
    HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x08, 1, &data, 1, 1000);
    HAL_Delay(10);

    // FIFO Pointer
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x04, 1, &data, 1, 1000);
    HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x05, 1, &data, 1, 1000);
    HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x06, 1, &data, 1, 1000);

    // Sampling Set - Sampling Rate : 200Hz, Pulse Width : 411μs
    data = 0x0B;  // 0b00001011
    HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x0A, 1, &data, 1, 1000);
    HAL_Delay(10);

    // LED Current set
    data = 0x10;  // MAX 50mA (0xFF) // 0x20: 6.4mA
    HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x0C, 1, &data, 1, 1000);
    HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x0D, 1, &data, 1, 1000);
    HAL_Delay(10);

    // Mode Set
    data = 0x03;  // 0x07 : Shutdown, 0x01 : IR, 0x02 : RED, 0x03 : Multi
    HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x09, 1, &data, 1, 1000);
    HAL_Delay(10);

    // Part ID= 0x15
    HAL_I2C_Mem_Read(&hi2c1, W_addr, 0xFF, 1, &data, 1, 1000);
    printf("Part ID: 0x%02X\r\n", data);  // Default 0x15
}*/

void Moduleset()
{
    // Reset
    data = 0x40;
    HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x09, 1, &data, 1, 1000);
    while(data & 0x40) {
    	HAL_I2C_Mem_Read(&hi2c1, R_addr, 0x09, 1, &data, 1, 1000);
    }
    HAL_Delay(10);

    // Interrupt Enable
    data = 0x40;  //
    HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x02, 1, &data, 1, 1000);
    HAL_Delay(10);

    // FIFO Set
    data = 0x70;  // 4 Sample Average: 32, RollOver Enable, Full Data Interrupt
    HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x08, 1, &data, 1, 1000);
    HAL_Delay(10);

    // FIFO Pointer
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x04, 1, &data, 1, 1000);
    HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x05, 1, &data, 1, 1000);
    HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x06, 1, &data, 1, 1000);

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

    // Interrupt Clear
    HAL_I2C_Mem_Read(&hi2c1, R_addr, 0x00, 1, &data, 1, 1000);
    HAL_I2C_Mem_Read(&hi2c1, R_addr, 0x01, 1, &data, 1, 1000);

    // Mode Set
    data = 0x07;  // 0x07 : Shutdown, 0x01 : IR, 0x02 : RED, 0x03 : Multi
    HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x09, 1, &data, 1, 1000);
    HAL_Delay(10);

    // Slot Set
	data = 0x21;  // Slot1 : RED, Slot2 : IR
	HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x11, 1, &data, 1, 1000);
	HAL_Delay(10);

    // Part ID= 0x15
    HAL_I2C_Mem_Read(&hi2c1, W_addr, 0xFF, 1, &data, 1, 1000);
    printf("Part ID: 0x%02X\r\n", data);  // Default 0x15
}

uint8_t getUnreadSampleCount()
{
    uint8_t wr = 0, rd = 0;
    HAL_I2C_Mem_Read(&hi2c1, R_addr, 0x04, 1, &wr, 1, 10);
    HAL_I2C_Mem_Read(&hi2c1, R_addr, 0x06, 1, &rd, 1, 10);
    if ((wr - rd) < 0)
        return wr - rd + 32;
    else
        return wr - rd;
}

/*
void MAX30102_ReadHeartRate()
{
    uint8_t readPointer, writePointer;
    uint32_t redData, irData;

    // FIFO Pointer Location
    HAL_I2C_Mem_Read(&hi2c1, R_addr, 0x06, 1, &readPointer, 1, 1000);
    HAL_I2C_Mem_Read(&hi2c1, R_addr, 0x04, 1, &writePointer, 1, 1000);

    // Data Number
    int Sample = writePointer - readPointer;
    if (Sample < 0) Sample += 32; // MAX Number : 32

    // Data read
    if (Sample > 0) {
        uint8_t dataBuffer[6];

        // FIFO Data Read
        HAL_I2C_Mem_Read(&hi2c1, R_addr, 0x07, 1, dataBuffer, 6, 1000);

        // Data Transform
        redData = ((uint32_t)dataBuffer[0] << 16 | (uint32_t)dataBuffer[1] << 8 | dataBuffer[2]) & 0x03FFFF;
        irData = ((uint32_t)dataBuffer[3] << 16 | (uint32_t)dataBuffer[4] << 8 | dataBuffer[5]) & 0x03FFFF;

        printf("Heart Rate : %d\r\n", redData);
        //printf("\r\SpO2 Rate : %d\r\n", redData);
    }
}
*/

/*
#define BUFF_SIZE 50

uint8_t heartRate = 0, spo2 = 0;
uint16_t eachSampleDiff = 0;

uint32_t red_sampleBuffer[BUFF_SIZE];
uint32_t iRed_sampleBuffer[BUFF_SIZE];

void buffInsert(uint32_t red, uint32_t iRed)
{
    uint8_t i;
    for (i = BUFF_SIZE - 1; i > 0; i--)
    {
    	red_sampleBuffer[i] = red_sampleBuffer[i - 1];
    	iRed_sampleBuffer[i] = iRed_sampleBuffer[i - 1];
    }
    red_sampleBuffer[0] = red;
    iRed_sampleBuffer[0] = iRed;
}

uint16_t redAC = 0;
uint32_t redDC = 0;
uint16_t iRedAC = 0;
uint32_t iRedDC = 0;

void calAcDc(uint16_t *rac, uint32_t *rdc, uint16_t *iac, uint32_t *idc)
{
    uint32_t rMax = red_sampleBuffer[0];
    uint32_t rMin = red_sampleBuffer[0];
    uint32_t iMax = iRed_sampleBuffer[0];
    uint32_t iMin = iRed_sampleBuffer[0];

    uint8_t i;
    for (i = 0; i < BUFF_SIZE; i++)
    {
        if (red_sampleBuffer[i] > rMax)
            rMax = red_sampleBuffer[i];
        if (red_sampleBuffer[i] < rMin)
            rMin = red_sampleBuffer[i];
        if (iRed_sampleBuffer[i] > iMax)
            iMax = iRed_sampleBuffer[i];
        if (iRed_sampleBuffer[i] < iMin)
            iMin = iRed_sampleBuffer[i];
    }
    *rac = rMax - rMin;
    *rdc = (rMax + rMin) / 2;
    *iac = iMax - iMin;
    *idc = (iMax + iMin) / 2;
}

#define FILTER_LEVEL 8
void filter(uint32_t *red, uint32_t *iRed)
{
    uint32_t red_t = 0;
    uint32_t iRed_t = 0;
    for (int i = 0; i < FILTER_LEVEL - 1; i++)
    {
    	red_t += red_sampleBuffer[i];
        iRed_t += iRed_sampleBuffer[i];
    }
    *red = (red_t + *red) / FILTER_LEVEL;
    *iRed = (iRed_t + *iRed) / FILTER_LEVEL;
}

void MAX30102_ReadHeartRate()
{
    uint32_t redData = 0;
    int unreadSampleCount = getUnreadSampleCount(); // 1 ~ 32

    // FIFO Pointer Location
    //HAL_I2C_Mem_Read(&hi2c1, R_addr, 0x06, 1, &readPointer, 1, 1000);
    //HAL_I2C_Mem_Read(&hi2c1, R_addr, 0x04, 1, &writePointer, 1, 1000);

    // Data Number
    //int Sample = writePointer - readPointer;
    //if (Sample < 0) Sample += 32; // MAX Number : 32

    // Data read
    //if (Sample > 0) {
    uint8_t temp_dataBuffer[6 * unreadSampleCount];
    uint32_t red_dataBuffer[unreadSampleCount];
    uint32_t iRed_dataBuffer[unreadSampleCount];

        // FIFO Data Read
        HAL_I2C_Mem_Read(&hi2c1, R_addr, 0x07, 1, temp_dataBuffer, 6 * unreadSampleCount, 1000);

        // Data Transform
        for(int i = 0; i < unreadSampleCount; i++) {
        	red_dataBuffer[i] += (((uint32_t)temp_dataBuffer[(6 * i) + 0] << 16) | ((uint32_t)temp_dataBuffer[(6 * i) + 1] << 8) | (uint32_t)temp_dataBuffer[(6 * i) + 2]) & 0x03FFFF;
        	iRed_dataBuffer[i] += (((uint32_t)temp_dataBuffer[(6 * i) + 3] << 16) | ((uint32_t)temp_dataBuffer[(6 * i) + 4] << 8) | (uint32_t)temp_dataBuffer[(6 * i) + 5]) & 0x03FFFF;
        }

        static uint8_t eachBeatSampleCount = 0;
        static uint8_t lastTenBeatSampleCount[10];
        static uint32_t last_iRed = 0;

        for (int i = 0; i < unreadSampleCount; i++)
            {
                if (iRed_dataBuffer[i] < 40000) //?��?��?��不�?�算，跳�?
                {
                    heartRate = 0;
                    spo2 = 0;
                    eachSampleDiff = 0;
                    continue;
                }
                buffInsert(red_dataBuffer[i], iRed_dataBuffer[i]);
                calAcDc(&redAC, &redDC, &iRedAC, &iRedDC);
                filter(&red_dataBuffer[i], &iRed_dataBuffer[i]);
                //计算spo2
                float R = (((float)(redAC)) / ((float)(redDC))) / (((float)(iRedAC)) / ((float)(iRedDC)));
                printf("R : %f\r\n", R);
                if (R >= 0.36 && R < 0.66){
                	spo2 = (uint8_t)(107 - 20 * R);
                	printf("spo2-1 : %d\r\n", spo2);
                }
                else if (R >= 0.66 && R < 1) {
                	spo2 = (uint8_t)(129.64 - 54 * R);
                	printf("spo2-2 : %d\r\n", spo2);
                }
                //计算心率,30-250ppm  count:200-12
                eachSampleDiff = last_iRed - iRed_dataBuffer[i];
                if (eachSampleDiff > 50 && eachBeatSampleCount > 12)
                {
                    for (int j = 9; j > 0; j--) {
                    	lastTenBeatSampleCount[i] = lastTenBeatSampleCount[i - 1];
                    }
                    lastTenBeatSampleCount[0] = eachBeatSampleCount;
                    uint32_t totalTime = 0;
                    for (int j = 0; j < 10; j++) {
                    	totalTime += lastTenBeatSampleCount[i];
                    }
                    heartRate = (uint8_t)(60.0 * 10 / 0.02 / ((float)totalTime));
                    eachBeatSampleCount = 0;
                }
                last_iRed = iRed_dataBuffer[i];
                eachBeatSampleCount++;
            }

        //redData = ((redData_h << 16) | (redData_m << 8) | redData_l) & 0x03FFFF;

        //printf("Heart Rate : %d\r\n", heartRate);
        //printf("spo2 : %d\r\n", spo2);
    //}
}*/
int last_ms = 0, BPM = 0;
void MAX30102_ReadHeartRate()
{
    uint8_t wr = 0, rd = 0;
    int now_ms = HAL_GetTick();
    int unreadSampleCount = getUnreadSampleCount(); // 1 ~ 32
    int beatCount = 0;
    uint32_t red_dataBuffer[unreadSampleCount];
    uint8_t temp_dataBuffer[3 * unreadSampleCount];

    //printf("unreadSampleCount: %d\r\n", unreadSampleCount);

    // Operating Check
	/*HAL_I2C_Mem_Read(&hi2c1, R_addr, 0x04, 1, &wr, 1, 10);
	HAL_I2C_Mem_Read(&hi2c1, R_addr, 0x06, 1, &rd, 1, 10);
	printf("wr1: %d, rd1: %d\r\n", wr, rd);*/

    // FIFO Data Read
    HAL_I2C_Mem_Read(&hi2c1, R_addr, 0x07, 1, temp_dataBuffer, 3 * unreadSampleCount, 1000);

    // Data Transform
	for(int i = 0; i < unreadSampleCount; i++) {
		red_dataBuffer[i] = (((uint32_t)temp_dataBuffer[(3 * i) + 0] << 16) | ((uint32_t)temp_dataBuffer[(3 * i) + 1] << 8) | (uint32_t)temp_dataBuffer[(3 * i) + 2]) & 0x03FFFF;
	}

    // Operating Check
    /*HAL_I2C_Mem_Read(&hi2c1, R_addr, 0x04, 1, &wr, 1, 10);
	HAL_I2C_Mem_Read(&hi2c1, R_addr, 0x06, 1, &rd, 1, 10);
	printf("wr2: %d, rd2: %d\r\n\r\n", wr, rd);*/

	for(int i = 0; i < unreadSampleCount; i++) {
		printf("red_data_%d: %d\r\n", i, red_dataBuffer[i]);
		if(red_dataBuffer[i] > 15000) {
			beatCount++;
		}
	}
	//printf("\r\n");

	//printf("beatCount: %d, time: %dms\r\n", beatCount, (now_ms - last_ms));

    BPM = (beatCount * 1000 * 60) / (now_ms - last_ms);
    //printf("BPM: %d\r\n", BPM);

    last_ms = now_ms;
    beatCount = 0;
}

int getIR() {
	uint8_t temp_dataBuffer[6];

	HAL_I2C_Mem_Read(&hi2c1, R_addr, 0x07, 1, temp_dataBuffer, 6, 1000);

	return (((uint32_t)temp_dataBuffer[0] << 16) | ((uint32_t)temp_dataBuffer[1] << 8) | (uint32_t)temp_dataBuffer[2]) & 0x03FFFF;
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  ProgramStart("Pulse Oximeter Test");
  i2c_scan();
  Moduleset();

  // LED Current set
  data = 0x0A;  // MAX 50mA (0xFF) // 0x20: 14.2mA
  HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x0C, 1, &data, 1, 1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //uint8_t interrupt;
  int rates[RATE_SIZE] = {0, };
  int rateSpot = 0;
  int beatAvg = 0;
  int lastBeat = 0;
  //int cnt = 0;
  while (1)
    {
	  int IRValue = getIR();
	  /*cnt++;
	  if(cnt == 30) {
		  printf("%d\r\n", IRValue);
		  cnt = 0;
	  }*/

	  if(checkForBeat(IRValue)) {
		  int delta = HAL_GetTick() - lastBeat;
		  lastBeat = HAL_GetTick();

		  int beatsPerMinute = 60000 / delta;

		  if ((beatsPerMinute < 255) && (beatsPerMinute > 20)) {
			rates[rateSpot++] = beatsPerMinute; //Store this reading in the array
			rateSpot %= RATE_SIZE; //Wrap variable

			//Take average of readings
			for (int x = 0 ; x < RATE_SIZE ; x++){
				beatAvg += rates[x];
			}
			beatAvg /= RATE_SIZE;
			printf("IRValue: %5d, BPM: %3d, avg_BPM: %3d\r\n", IRValue, beatsPerMinute, beatAvg);
		  }
	  }
  	  /*HAL_I2C_Mem_Read(&hi2c1, R_addr, 0x00, 1, &interrupt, 1, 1000);
  	  //printf("interrupt: 0x%x\r\n", interrupt);
  	  if((interrupt & 0x40)) {
  		  MAX30102_ReadHeartRate();
  	  }
  	  //HAL_Delay(20);*/
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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

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
