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
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define BUF_SIZE 100
char buf1[BUF_SIZE], buf2[BUF_SIZE], temp_AT[BUF_SIZE]; // DMA Buffer
char dum1, dum2;
int head1 = 0, head2 = 0, tail1 = 0, tail2 = 0;
char stm_c[5] = { '0', }, app_c[5] = { '0', };
int stm_i = 0, app_i = 0;
int last_printe = 0;

int printe(char* str, int id) {
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
			sprintf(temp_AT, "%d,%d\r\n", id, str_len);
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

    		  char comp_buf[BUF_SIZE];
    		  sprintf(comp_buf, "%s", buf1);
    		  //printf("buf1: %s | comp_buf: %s", buf1, comp_buf);
    		  if(!strncmp(comp_buf + 2, "CONNECT", 7)) {
    			  app_c[app_i++] = buf1[0];
    			  printf("connect app.\r\n");
    		  }

    		  if(!strncmp(comp_buf, "+IPD,", 5)) {
    			  char temp_val[BUF_SIZE];
    			  switch(comp_buf[9]) {												// Recieve MSG From Client
    				  case '0':														// Emergency (Not use in Server)
    					  break;
    				  case '1':														// Origin is xxx (Not use in Server)
    					  // Origin is xxx
    					  break;
    				  case '2':														// Destination is xxx
    					  // recieve 2xxx value ==> Destination is xxx
						  sprintf(temp_val, "%c%c%c%c", comp_buf[9], comp_buf[10], comp_buf[11], comp_buf[12]);
						  for(int i = 0; i < app_i; i++) {
							  while(!printe(temp_val, app_c[i]));
						  }
    					  break;
    				  case '3':														// Arrive at Destination (Not use in Server)
    					  break;
    				  case '4':														// Hall Sensor
    					  if(comp_buf[10] == '0') {
    						  // Unbuckle seat belt
    						  printf("Unbuckle seat belt\r\n");
    					  }
    					  else if(comp_buf[10] == '1') {
    						  // Buckle seat belt
    						  printf("Buckle seat belt\r\n");
    					  }
    					  break;
    				  case '5':														// Force Sensor
    					  if(comp_buf[10] == '0') {
    						  // Pressure X
    						  printf("Pressure X\r\n");
    					  }
    					  else if(comp_buf[10] == '1') {
    						  // Pressure O
    						  printf("Pressure O\r\n");
    					  }
    					  break;
    				  case '6':														// Heart Beat Sensor
    					  if(comp_buf[10] == '0') {
    						  // no finger
    						  printf("no finger\r\n");
    					  }
    					  else if(comp_buf[10] == '1'){
    						  // recieve 61xx value ==> normal
    						  printf("normal\r\n");
    					  }
    					  else if(comp_buf[10] == '2'){
    						  // recieve 62xx value ==> abnormal
    						  printf("abnormal\r\n");
    					  }
    					  break;
    				  case '7':														// Voltage Sensor (Not use in Server)
    					  break;
    				  case '8':														// STM Client
    					  stm_c[stm_i++] = app_c[--app_i];
    					  app_c[app_i] = '0';
    					  printf("not connect app. connect stm.\r\n");
    					  break;
    			  }
    		  }

    		  if(!strncmp(comp_buf + 2, "CLOSED", 6)) {
    			  if((app_i > 0) && (app_c[app_i - 1] == buf1[0])) {
    				  app_c[--app_i] = '0';
    				  printf("disconnect app.\r\n");
    			  }
    			  else if((stm_i > 0) && (stm_c[stm_i - 1] == buf1[0])) {
    				  stm_c[--stm_i] = '0';
    				  printf("disconnect stm.\r\n");
    			  }
    		  }
    		  for(int i = 0; i < stm_i; i++) {
    			  printf("i: %d, stm content: %c\r\n", i, stm_c[i]);
    		  }
    		  for(int i = 0; i < app_i; i++) {
    			  printf("i: %d, app content: %c\r\n", i, app_c[i]);
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

void ESP8266_server_init(){
	/*HAL_UART_Transmit(&huart1, "AT+RST\r\n", 8, 10);					// RESET
	HAL_Delay(1000);*/

	HAL_UART_Transmit(&huart1, "AT+CWMODE=2", 11, 10);				// 1: CLIENT MODE, 2: SERVER MODE, 3: Multi mode
	HAL_UART_Transmit(&huart1, "\r\n", 2, 10);
	HAL_Delay(10);

	HAL_UART_Transmit(&huart1, "AT+CIPMUX=1", 11, 10);				// 0: single connection mode, 1: multi connection mode
	HAL_UART_Transmit(&huart1, "\r\n", 2, 10);
	HAL_Delay(10);

	HAL_UART_Transmit(&huart1, "AT+CIPSERVE", 11, 10);
	HAL_UART_Transmit(&huart1, "R=1,3000\r\n", 10, 10);		// SERVER setting (0: SERVER CLOSE, 1: SERVER OPEN)
	HAL_Delay(10);

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
  /* USER CODE BEGIN 2 */
  ProgramStart("ESP8266 Test - Start");
  UART_Start_Receive_IT(&huart1, &dum1, 1);
  UART_Start_Receive_IT(&huart2, &dum2, 1);
  //ESP8266_server_init();

  /* USER CODE END 2 */

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
  huart1.Init.BaudRate = 115200;
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
