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
#include "heartRate.h"
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
#define RATE_SIZE 10

uint16_t W_addr = 0xAE;
uint16_t R_addr = 0xAF;
uint8_t data;

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
  ProgramStart("HeartRate Start!\r\n");
  i2c_scan();
  Moduleset();

  // LED Current set
  data = 0x0A;  // MAX 50mA (0xFF) // 0x20: 14.2mA
  HAL_I2C_Mem_Write(&hi2c1, W_addr, 0x0C, 1, &data, 1, 1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int rates[RATE_SIZE] = {0, };
  int rateSpot = 0;
  double beatsPerMinute = 0;
  int beatAvg = 0;
  int lastBeat = 0;
  while (1)
  {
	  HAL_I2C_Mem_Read(&hi2c1, R_addr, 0x00, 1, &data, 1, 1000);
	  if(data & 0x40) {
		  int IRValue = getIR();

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
		  }
		  printf("IRValue: %5d, BPM: %5.2f, avg_BPM: %3d\r\n", IRValue, beatsPerMinute, beatAvg);
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
