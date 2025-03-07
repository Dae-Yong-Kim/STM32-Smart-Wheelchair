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
DMA_HandleTypeDef hdma_i2c1_rx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
		if((addr % 16) == 15)	printf("\r\n");
	}
}

uint16_t Gyro_addr = 0xD0;
uint8_t data;

void ModuleSet()
{
	// PWR_MGMT : 0x6B
	data = 0x80;	// Reset
	HAL_I2C_Mem_Write(&hi2c1, Gyro_addr, 0x6B, 1, &data, 1, 1000);
	HAL_Delay(100);

	// PWR_MGMT : 0x6B
	data = 0x00;	// Operating
	HAL_I2C_Mem_Write(&hi2c1, Gyro_addr, 0x6B, 1, &data, 1, 1000);
	HAL_Delay(100);

	// Signal Path Reset : 0x68
	data = 0x07;	// All Reset
	HAL_I2C_Mem_Write(&hi2c1, Gyro_addr, 0x68, 1, &data, 1, 1000);
	HAL_Delay(100);

	// Sampling Rate set : 0x19
	data = 0x00;	// 1kHz Sampling Rate
	HAL_I2C_Mem_Write(&hi2c1, Gyro_addr, 0x19, 1, &data, 1, 1000);
	HAL_Delay(100);

	// Gyroscope set : 0x1B
	data = 0x00;	// -250 ~ +250 Degree/sec
	HAL_I2C_Mem_Write(&hi2c1, Gyro_addr, 0x1B, 1, &data, 1, 1000);
	HAL_Delay(100);

	// Accelerometer set : 0x1C
	data = 0x18;	// -16 ~ +16 g
	HAL_I2C_Mem_Write(&hi2c1, Gyro_addr, 0x1C, 1, &data, 1, 1000);
	HAL_Delay(100);

	// Accelerometer LPF set : 0x1D
	data = 0x00;	// 460 Hz LPF
	HAL_I2C_Mem_Write(&hi2c1, Gyro_addr, 0x1D, 1, &data, 1, 1000);
	HAL_Delay(100);
}

unsigned char gyro[6];
void ReadGyro()
{
   // current Time Measure
   //uint32_t currentTime = HAL_GetTick();

   // GYRO Data Read : 0x43  XH-XL-YH-YL-ZH-ZL
   HAL_I2C_Mem_Read(&hi2c1, Gyro_addr, 0x43, 1, gyro, 6, 1000);

   short gyro_x = (gyro[0]<<8) + gyro[1];
   short gyro_y = (gyro[2]<<8) + gyro[3];
   short gyro_z = (gyro[4]<<8) + gyro[5];

   double gX = (double)gyro_x / 131.0;   // degree per second Value
   double gY = (double)gyro_y / 131.0;
   double gZ = (double)gyro_z / 131.0;

  printf("gyroX : %2.1f, gyroY : %2.1f, gyroZ : %2.1f\r\n", gX, gY, gZ);
}

unsigned char acc[6];
void ReadAcc()
{

   // GYRO Data Read : 0x3B  XH-XL-YH-YL-ZH-ZL
   HAL_I2C_Mem_Read(&hi2c1, Gyro_addr, 0x3B, 1, acc, 6, 1000);

   short acc_x = (acc[0]<<8) + acc[1];
   short acc_y = (acc[2]<<8) + acc[3];
   short acc_z = (acc[4]<<8) + acc[5];

   // -2 ~ 2
   //   double aX = acc_x / 16384;   // g Value
   //   double aY = acc_y / 16384;
   //   double aZ = acc_z / 16384;

   // -16 ~ 16
   double aX = (double)acc_x / 2048.0;   // g Value
   double aY = (double)acc_y / 2048.0;
   double aZ = (double)acc_z / 2048.0;

     printf("aX : %2.1f, aY : %2.1f, aZ : %2.1f\r\n", aX, aY, aZ);
}

double radTodeg = 180/3.141592;
double angleX = 0.0;  // Roll
double angleY = 0.0;  // Pitch

void ReadAcc_Angle()
{

   // GYRO Data Read : 0x3B  XH-XL-YH-YL-ZH-ZL
   HAL_I2C_Mem_Read(&hi2c1, Gyro_addr, 0x3B, 1, acc, 6, 1000);

   short acc_x = (acc[0]<<8) + acc[1];
   short acc_y = (acc[2]<<8) + acc[3];
   short acc_z = (acc[4]<<8) + acc[5];

   printf("Raw: X=%d, Y=%d, Z=%d\r\n", acc_x, acc_y, acc_z);

<<<<<<< HEAD
	// 2?�� 보수 처리 (�??�� ?��?�� 16비트 값으�? �??��)
	if(acc_x > 32767) acc_x -= 65536;
	if(acc_y > 32767) acc_y -= 65536;
	if(acc_z > 32767) acc_z -= 65536;
=======
   // -16 ~ 16
   double aX = (double)acc_x / 2048.0;   // g Value
   double aY = (double)acc_y / 2048.0;
   double aZ = (double)acc_z / 2048.0;
>>>>>>> 250673826ab0043e1ea6586ae066b72bd85eedae

   printf("g: X=%.3f, Y=%.3f, Z=%.3f\r\n", aX, aY, aZ);

   // Roll
   angleX = atan2(aY, sqrt(pow(aX, 2) + pow(aZ, 2))) * radTodeg;
   // Pitch
   angleY = atan2(-aX, sqrt(pow(aY, 2) + pow(aZ, 2))) * radTodeg;

   printf("Roll (X): %3.1f, Pitch (Y): %3.1f\r\n", angleX, angleY);

<<<<<<< HEAD
	// Roll
	angleX = atan2(aY, sqrt(pow(aX, 2) + pow(aZ, 2))) * radTodeg;
	// Pitch
	angleY = atan2(-aX, sqrt(pow(aY, 2) + pow(aZ, 2))) * radTodeg;

	printf("Roll (X): %3.1f, Pitch (Y): %3.1f\r\n\n", angleX, angleY);


//	double accel_yz = sqrt(pow(aY,2)+pow(aZ,2));
//	angleY = atan(-aX/accel_yz)*radTodeg;
=======
//   double accel_yz = sqrt(pow(aY,2)+pow(aZ,2));
//   angleY = atan(-aX/accel_yz)*radTodeg;
>>>>>>> 250673826ab0043e1ea6586ae066b72bd85eedae
//
//   double accel_xz = sqrt(pow(aX,2)+pow(aZ,2));
//   angleX = atan(aY/accel_xz)*radTodeg;
//
//   double accel_xy = sqrt(pow(aX,2)+pow(aY,2));
//   angleZ = atan(accel_xy/aZ)*radTodeg;

  //printf("angleX : %2.1f, angleY : %2.1f, angleZ : %2.1f\r\n", angleX, angleY, angleZ);
}

double gyroZbias = 0.0; // Gyro Noise
// Gyro Standard Noise Detect
void CalibrateGyro()
{
    int sum = 0;
    int samples = 10;

    for(int i = 0; i < samples; i++)
    {
        HAL_I2C_Mem_Read(&hi2c1, Gyro_addr, 0x47, 1, gyro, 2, 1000);
        short gZ = (gyro[0]<<8) + gyro[1];
        //gZ = gZ / 131.0;
        if((gZ > 0.05) || (gZ < -150))	sum += 0;
        else							sum += gZ;
        HAL_Delay(10);
    }
    double devide = (double)samples * 131;
    gyroZbias = (double)sum / devide;
    printf("gyroZbias : %7.3f\r\n", gyroZbias);
    HAL_Delay(10);
}

unsigned char gy_z[2];
double pre_gZ = 0, total_gZ = 0;
int before = 0;
double max_degree = 0;
void Read_Z_Angle()
{
	CalibrateGyro();

	HAL_I2C_Mem_Read(&hi2c1, Gyro_addr, 0x47, 1, gy_z, 2, 1000);

	short gyro_z = (gy_z[0]<<8) + gy_z[1];
	double gZ = (double)gyro_z / 131.0;


	gZ -= gyroZbias;

	total_gZ += (gZ + pre_gZ) * (HAL_GetTick() - before) / 2000;

	double gyroRate = gZ;
	double gyroAngle = total_gZ;

	printf("gyrorate: %7.3f, gyroAngle: %7.3f\r\n", gyroRate, gyroAngle);

	// 상보필터
	if (fabs(gyroRate) < 0.05) {
		// 움직임이 거의 없을 때 드리프트 감소
		total_gZ = pre_gZ * 0.999 + gyroAngle * 0.001;
	} else {
		// 움직임이 있을 때는 자이로 데이터에 더 의존
		total_gZ = gyroAngle * 0.999 + pre_gZ * 0.001;
	}

	printf("gZ: %7.3f | total z degree: %7.3f\r\n", gZ, total_gZ);

	pre_gZ = gZ;
	before = HAL_GetTick();

	if(fabs(max_degree) < fabs(total_gZ))	max_degree = total_gZ;
	printf("max_degree : %7.3f\r\n", max_degree);
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	ProgramStart("Gyroscope Test");
	i2c_scan();
	ModuleSet();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  //ReadGyro();
	  Read_Z_Angle();
	  HAL_Delay(30);
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

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
