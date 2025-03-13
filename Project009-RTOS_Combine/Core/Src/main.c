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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

osThreadId Task1Handle;
osThreadId Task2Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
void StartTask1(void const * argument);
void StartTask02(void const * argument);

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
//		  case 0:		// Slow Stop
//			  // Task1
//			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
//			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
//			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
//			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
//			  break;
		  case 0:		// Emergency Stop
			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
			  break;
		  case 1:		// Forward
			  htim1.Instance->CCR1 = 1400;
			  htim1.Instance->CCR3 = 1400;

			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
			  break;
		  case 2:		// Backward
			  htim1.Instance->CCR1 = 1400;
			  htim1.Instance->CCR3 = 1400;
			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
			  break;
		  case 3:		// Slow Right
			  htim1.Instance->CCR1 = 1600;		//ENA
			  htim1.Instance->CCR3 = 1300;		//ENB

			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
			  break;
		  case 4:		// Quick Right
			  htim1.Instance->CCR1 = 1400;
			  htim1.Instance->CCR3 = 1400;

			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
			  break;
		  case 5:		// Slow Left
			  htim1.Instance->CCR1 = 1300;		//ENA
			  htim1.Instance->CCR3 = 1600;		//ENB

			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
			  break;
		  case 6:		// Quick Left
			  htim1.Instance->CCR1 = 1400;
			  htim1.Instance->CCR3 = 1400;

			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
			  break;
	}
}

int Mcntr = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin) {
	case GPIO_PIN_13:
		if(Mcntr > 6)	Mcntr = 0;
		else			Mcntr++;
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

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);	// Motor PWM1
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);	// Motor PWM2

	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);	// Ultrasonic FF_Echo
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);	// Ultrasonic BB_Echo
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);	// Ultrasonic FR_Echo
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);	// Ultrasonic FL_Echo

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 84-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim5.Init.Period = 60000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
unsigned char str1[10];
unsigned char str2[100];

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask1 */
/**
  * @brief  Function implementing the Task1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask1 */

// Motor Mode Task
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
	  if((FF_dist < 100) && (FR_dist < 100) && (FL_dist < 100))
	  {
		  Mcntr = 2;
	  }
	  else if((FF_dist < 100) && (FR_dist < 100))
	  {
		  Mcntr = 6;
	  }
	  else if((FF_dist < 100) && (FL_dist < 100))
	  {
		  Mcntr = 4;
	  }
	  else if(FF_dist < 100)
	  {
		  Mcntr = 0;
	  }
	  else if(FR_dist < 100)
	  {
		  Mcntr = 5;
	  }
	  else if(FL_dist < 100)
	  {
		  Mcntr = 3;
	  }
	  else
	  {
		  Mcntr = 1;
	  }

	  /* Debug */
	  sprintf(str1,"%d\r\n",Mcntr);
	  HAL_UART_Transmit(&huart2, str1, 10, 1000);

	  sprintf(str2,"FF:%.2f, FR:%.2f, FL:%.2f, FB:%.2f\r\n",FF_dist, FR_dist, FL_dist, BB_dist);
	  HAL_UART_Transmit(&huart2, str2, 100, 1000);

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

// Ultrasonic calculation
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
