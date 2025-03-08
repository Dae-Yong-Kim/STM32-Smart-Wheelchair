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

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
					  spd -= 30;
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
			  htim1.Instance->CCR1 = 300;		//ENA
			  htim1.Instance->CCR3 = 100;		//ENB

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
			  htim1.Instance->CCR1 = 100;		//ENA
			  htim1.Instance->CCR3 = 300;		//ENB

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

int mode = 0;
// 0 : standby, 1 : Passenger, 2 : watchdog
// 3 : Obstacle, 4 : Forward First, 5 : Right First, 6 : Left First
// 7 : Destination, 8 : Charge
void Move_mode()
{
	switch(mode)
	{
	case 0 :	// Standby

		break;
	case 1 :	// Passenger

		break;
	case 2 : 	// Watchdog

		break;
	case 3 :	// Obstacle

		break;
	case 4 :	// Forward First
		if(FF_dist == -1 || FR_dist == -1 || FL_dist == -1) {}
		else if(FF_dist < 400 && FR_dist < 400 && FL_dist < 400) {	// MoveBackWard
		  Motor_Mode(0);		// Stop
		  HAL_Delay(50);
		  Motor_Mode(2);		// Rear
		  HAL_Delay(500);
		  if(FR_dist < FL_dist){
			  Motor_Mode(3);	// Quick Left
			  HAL_Delay(800);
		  }
		  else if(FR_dist > FL_dist){
			  Motor_Mode(5);	// Quick Right
			  HAL_Delay(800);
		  }
		}
		else if((FR_dist < 400) && (FR_dist < FL_dist)){
		  Motor_Mode(4); // Soft Left
		  HAL_Delay(800);
		}
		else if((FL_dist < 400) && (FR_dist > FL_dist)){
		  Motor_Mode(6); // Soft right
		  HAL_Delay(800);
		}
		else{
		  Motor_Mode(1);		// Front
		}
		break;
	case 5 : 	// Right First

		break;
	case 6 :	// Left First

		break;
	case 7 :	// Destination


		break;
	case 8 :	// Charge

		break;
	}
}


double FF_dist = 0, FR_dist = 0, FL_dist = 0, BB_dist = 0;

// GPIO Interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin) {
  case FFecho_Pin:
	  if(HAL_GPIO_ReadPin(FFecho_GPIO_Port, FFecho_Pin)) {
		  htim2.Instance->CNT = 0;
	  }
	  else {
		  if(htim2.Instance->CNT > 60000) {
			  FF_dist = -1;
		  }
		  else {
			  FF_dist = htim2.Instance->CNT * 0.17;
		  }
	  }
	  break;
  case FRecho_Pin:
  	  if(!HAL_GPIO_ReadPin(FRecho_GPIO_Port, FRecho_Pin)) {
  		if(htim2.Instance->CNT > 60000) {
  			FR_dist = -1;
  		}
  		else {
  			FR_dist = htim2.Instance->CNT * 0.17;
  		}
  	  }
  	  break;
	case FLecho_Pin:
  	  if(!HAL_GPIO_ReadPin(FLecho_GPIO_Port, FLecho_Pin)) {
  		if(htim2.Instance->CNT > 60000) {
			FL_dist = -1;
		  }
		  else {
			FL_dist = htim2.Instance->CNT * 0.17;
		  }
  	  }
  	  break;
	case BBecho_Pin:
	  if(!HAL_GPIO_ReadPin(BBecho_GPIO_Port, BBecho_Pin)) {
		if(htim2.Instance->CNT > 60000) {
			BB_dist = -1;
		  }
		  else {
			BB_dist = htim2.Instance->CNT * 0.17;
		  }
	  }
}

#define BUF_SIZE 100
char buf1[BUF_SIZE], buf2[BUF_SIZE]; // DMA Buffer
char dum1, dum2;
int head1 = 0, head2 = 0, tail1 = 0, tail2 = 0, temp1 = 0, temp2 = 0; // mode 0: AT command, 1: regularly send AT+INQ and detect entered slave address
int sn; //slave number
char* slave_addr[5] = {"8E4591", "15DA51", "37826F", "A7EF18", "9B0C60"}; // slave address

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   if(huart == &huart1)
   {
      buf1[tail1++] = dum1;
      if(!mode) {
    	  HAL_UART_Transmit(&huart2, &dum1/*== buf1+t1-1*/, 1, 10);      // putty print
      }
      if(dum1 == '\r')
      {
    	  if(mode){
			  char comp_buf[BUF_SIZE];
			  if(tail1 > 15) {
				  sprintf(comp_buf, "%s\n\0", &buf1[tail1 - 7]);
				  if(!strncmp(comp_buf, slave_addr[sn], 6)) {
					  /* Add to Mode select */
					  printf("I found it%d\r\n\r\n", temp2++);
				  }
			  }
    	  }
         tail1 = 0;
      }
      HAL_UART_Receive_IT(&huart1, &dum1, 1);         // interrupt chain
   }
   /* Debugging */
//   else if(huart == &huart2)
//   {
//      buf2[tail2++] = dum2;
//      HAL_UART_Transmit(&huart2, &dum2, 1, 10); // terminal echo
//      if(dum2 == '\r')  // CR : 0x0d
//      {
//         HAL_UART_Transmit(&huart2, "\n", 1, 10); // terminal echo
//         buf2[tail2++] = '\n'; // == HAL_UART_Transmit(&huart1, "\n", 1, 10);
//         HAL_UART_Transmit(&huart1, buf2, tail2, 10);   // AT Command
//         tail2 = 0;
//      }
//      HAL_UART_Receive_IT(&huart2, &dum2, 1);
//   }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	HAL_UART_Transmit(&huart1, "AT+INQ\r\n", 8, 10);
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
  /* USER CODE BEGIN 2 */

    ProgramStart("Motor + Bluetooth + Usonic + Gyro");
  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);	// Motor PWM1
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);	// Motor PWM2
	htim1.Instance->CCR1 = 300;
	htim1.Instance->CCR3 = 300;
//	htim1.Instance->CCR1 = htim1.Instance->ARR / 3;
//	htim1.Instance->CCR3 = htim1.Instance->ARR / 3;
	HAL_TIM_Base_Start(&htim2);
	UART_Start_Receive_IT(&huart1, &dum1, 1);
	UART_Start_Receive_IT(&huart2, &dum2, 1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);	// Ultrasonic Trig
	HAL_TIM_Base_Start_IT(&htim4);			// bluetooth


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  printf("FF_dist = %.2f, FR_dist = %.2f, FL_dist = %.2f, BB_dist = %.2f\r\n", FF_dist, FR_dist, FL_dist, BB_dist);
	  Motor_Mode(0);
	  HAL_Delay(500);
//	  if(FF_dist == -1 || FR_dist == -1 || FL_dist == -1) {}
//	  else if(FF_dist < 400 && FR_dist < 400 && FL_dist < 400) {	// MoveBackWard
//		  Motor_Mode(0);		// Stop
//		  HAL_Delay(50);
//		  Motor_Mode(2);		// Rear
//		  HAL_Delay(500);
//		  if(FR_dist < FL_dist){
//			  Motor_Mode(3);	// Quick Left
//			  HAL_Delay(800);
//		  }
//		  else if(FR_dist > FL_dist){
//			  Motor_Mode(5);	// Quick Right
//			  HAL_Delay(800);
//		  }
//	  }
//	  else if((FR_dist < 400) && (FR_dist < FL_dist)){
//		  Motor_Mode(4); // Soft Left
//		  HAL_Delay(800);
//	  }
//	  else if((FL_dist < 400) && (FR_dist > FL_dist)){
//		  Motor_Mode(6); // Soft right
//		  HAL_Delay(800);
//	  }
//  //	  else if(){
//  //		  mode = 3;		// Rear
//  //	  }
//  //	  else if(FF_dist < 200 && FR_dist < 200){
//  //		  mode = 4;		// Quick Left
//  //	  }
//  //	  else if(FR_dist < 200){
//  //		  mode = 5;		// Soft Left
//  //	  }
//  //	  else if(FF_dist < 100 && FL_dist < 100){
//  //		  mode = 6;		// Quick Right
//  //	  }
//  //	  else if(FL_dist < 100){
//  //		  mode = 7;		// Soft Right
//  //	  }
//	  else{
//		  Motor_Mode(1);		// Front
//	  }
//
//	  //HAL_UART_Transmit(&huart1, "AT+INQ\r\n", 8, 10);
//	  HAL_Delay(1000);


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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 60000-1;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10-1;
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
  htim4.Init.Period = 20000-1;
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

  /*Configure GPIO pins : FLecho_Pin FRecho_Pin */
  GPIO_InitStruct.Pin = FLecho_Pin|FRecho_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_Pin IN4_Pin IN2_Pin IN3_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN4_Pin|IN2_Pin|IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BBecho_Pin */
  GPIO_InitStruct.Pin = BBecho_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BBecho_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FFecho_Pin */
  GPIO_InitStruct.Pin = FFecho_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FFecho_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

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
