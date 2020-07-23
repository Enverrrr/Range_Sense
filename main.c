/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include <stdio.h>
#include <string.h>
/*
#define at "AT\r\n"
#define at_cıpshut "AT+CIPSHUT\r\n"
#define AT_CPIN? "AT+CPIN?\r\n"
#define AT_CSQ      "AT+CSQ\r\n"
#define AT_CREG?    "AT+CREG?\r\n"
#define AT_CGATT?   "AT+CGATT\r\n"
#define AT_CSTT     "AT+CSTT=\"internet\"\r\n"
#define AT_CIICR "AT+CIICR\r\n"
#define AT_CIFSR "AT+CIFSR\r\n"
#define AT_CIPSTART "AT+CIPSTART=\"TCP\",\"185.122.200.220\",\"9090\"\r\n"
#define AT_CIPSEND "AT+CIPSEND\r\n"
*/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
extern uint8_t Range_data[4];
extern uint8_t data_valid;
extern uint8_t receive_data;
extern uint8_t package_state;
extern uint8_t data_index;

uint8_t config=0;
uint16_t Result;
uint8_t ResultTxt[25];
uint8_t response[250];
uint8_t check=0;

extern uint16_t response_data[14];
extern uint8_t modem_valid;
extern uint8_t modem_receive;
extern uint8_t modem_package;
extern uint8_t modem_index;
uint8_t send_state=0;
uint8_t object=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Enter(void){
	uint8_t enter[2]={0x10,0x13};
	HAL_UART_Transmit(&huart4, enter, 2, 100);
}
void CTRL_Z(void){
	uint8_t ctrlz[2]={0x10,0x1A};
	HAL_UART_Transmit(&huart4, ctrlz, 2, 100);
}
void AwakenClient(void){
	uint8_t enter[2]={0x10,0x13};

	//HAL_UART_Transmit(&huart5, (uint8_t*)"ATE0\r\n", sizeof("ATE0\r\n"), 100);
//	HAL_UART_Transmit(&huart5, enter, 2, 100);
	//HAL_Delay(500);

	HAL_UART_Transmit(&huart5, (uint8_t*)"AT\n\r", sizeof("AT\n\r"), 100);
	HAL_UART_Transmit(&huart5, enter, 2, 100);
//	HAL_UART_Receive(&huart5, &response[0],4,100);
//	HAL_Delay(500);

	HAL_UART_Transmit(&huart5, (uint8_t*)"AT+CIPSHUT\n\r", sizeof("AT+CIPSHUT\n\r"), 100);
	HAL_UART_Transmit(&huart5, enter, 2, 100);
//	HAL_UART_Receive(&huart5, &response[17],14,100);
	//HAL_Delay(500);

	HAL_UART_Transmit(&huart5, (uint8_t*)"AT+CPIN?\n\r", sizeof("AT+CPIN?\n\r"), 100);
	HAL_UART_Transmit(&huart5, enter, 2, 100);
//	HAL_UART_Receive(&huart5, &response[2], 4,100);
//	HAL_Delay(500);

	//	HAL_UART_Transmit(&huart5, (uint8_t*)"AT+CIPSHUT\r\n", sizeof("AT+CIPSHUT\r\n"), 100);
	//	HAL_Delay(50);

	HAL_UART_Transmit(&huart5, (uint8_t*)"AT+CSQ\n\r", sizeof("AT+CSQ\n\r"), 100);
	HAL_UART_Transmit(&huart5, enter, 2, 100);
//	HAL_Delay(500);

	HAL_UART_Transmit(&huart5, (uint8_t*)"AT+CREG?\n\r", sizeof("AT+CREG?\n\r"), 100);
	HAL_UART_Transmit(&huart5, enter, 2, 100);
//	HAL_Delay(500);

	HAL_UART_Transmit(&huart5, (uint8_t*)"AT+CGATT?\n\r", sizeof("AT+CGATT?\n\r"), 100);
	HAL_UART_Transmit(&huart5, enter, 2, 100);
//	HAL_Delay(500);

	HAL_UART_Transmit(&huart5, (uint8_t*)"AT+CGATT=1\n\r", sizeof("AT+CGAT=1\n\r"), 100);
	HAL_UART_Transmit(&huart5, enter, 2, 100);
//	HAL_Delay(500);

	HAL_UART_Transmit(&huart5, (uint8_t*)"AT+CSTT=\"internet\"\n\r", sizeof("AT+CSTT=\"internet\"\n\r"), 100);
	HAL_UART_Transmit(&huart5, enter, 2, 100);
//	HAL_Delay(500);

	HAL_UART_Transmit(&huart5,(uint8_t*)"AT+CIICR\n\r", sizeof("AT+CIICR\n\r"),100);
	HAL_UART_Transmit(&huart5, enter, 2, 100);
//	HAL_Delay(500);

	HAL_UART_Transmit(&huart5,(uint8_t*)"AT+CIFSR\n\r", sizeof("AT+CIFSR\n\r"),100);
	HAL_UART_Transmit(&huart5, enter, 2, 100);
//	HAL_UART_Receive(&huart5, &response,23,100);
//	HAL_Delay(500);

}
void WriteServer(uint16_t Range){
	uint8_t Message[20];
	uint8_t enter[2]={0x10,0x13};
	uint8_t ctrlz[2]={0x10,0x1A};

	memset(Message,0,sizeof(Message));
	sprintf(Message,"%d\n\r",Range);

	HAL_UART_Transmit(&huart5, (uint8_t*)"AT\n\r", sizeof("AT\n\r"), 100);
	HAL_UART_Transmit(&huart5, enter,sizeof(enter), 100);
	HAL_Delay(400);
/*
	HAL_UART_Transmit(&huart5, (uint8_t*)"AT+CIPSHUT?\n\r", sizeof("AT+CIPSHUT\n\r"), 100);
	HAL_UART_Transmit(&huart5, enter,sizeof(enter), 100);
	HAL_Delay(100);
*/
	HAL_UART_Transmit(&huart5, (uint8_t*)"AT+CPIN?\n\r", sizeof("AT+CPIN?\n\r"), 100);
	HAL_UART_Transmit(&huart5, enter,sizeof(enter), 100);
	HAL_Delay(400);

	HAL_UART_Transmit(&huart5, (uint8_t*)"AT+CSQ\n\r", sizeof("AT+CSQ\n\r"), 100);
	HAL_UART_Transmit(&huart5, enter,sizeof(enter), 100);
	HAL_Delay(400);

	HAL_UART_Transmit(&huart5, (uint8_t*)"AT+CREG?\n\r", sizeof("AT+CREG?\n\r"), 100);
	HAL_UART_Transmit(&huart5, enter,sizeof(enter), 100);
	HAL_Delay(400);

	HAL_UART_Transmit(&huart5, (uint8_t*)"AT+CGATT?\n\r", sizeof("AT+CGATT?\n\r"), 100);
	HAL_UART_Transmit(&huart5, enter,sizeof(enter), 100);
	HAL_Delay(400);

	HAL_UART_Transmit(&huart5, (uint8_t*)"AT+CREG?\n\r", sizeof("AT+CREG?\n\r"), 100);
	HAL_UART_Transmit(&huart5, enter,sizeof(enter), 100);
	HAL_Delay(400);

	HAL_UART_Transmit(&huart5, (uint8_t*)"AT+CGATT?\n\r", sizeof("AT+CGATT?\n\r"), 100);
	HAL_UART_Transmit(&huart5, enter,sizeof(enter), 100);
	HAL_Delay(400);

	HAL_UART_Transmit(&huart5, (uint8_t*)"AT+CSTT=\"internet\"\n\r", sizeof("AT+CSTT=\"internet\"\n\r"), 100);
	HAL_UART_Transmit(&huart5, enter,sizeof(enter), 100);
	HAL_Delay(400);

	HAL_UART_Transmit(&huart5, (uint8_t*)"AT+CIICR\n\r", sizeof("AT+CIICR\n\r"), 100);
	HAL_UART_Transmit(&huart5, enter,sizeof(enter), 100);
	HAL_Delay(400);

	HAL_UART_Transmit(&huart5, (uint8_t*)"AT+CIFSR\n\r", sizeof("AT+CIFSR\n\r"), 100);
	HAL_UART_Transmit(&huart5, enter,sizeof(enter), 100);
	HAL_Delay(400);

	HAL_UART_Transmit(&huart5, (uint8_t*)"AT+CIPSTART=\"TCP\",\"185.122.200.220\",\"9090\"\n\r", sizeof("AT+CIPSTART=\"TCP\",\"185.122.200.220\",\"9090\"\n\r"), 100);
	HAL_UART_Transmit(&huart5, enter,sizeof(enter), 100);
	HAL_Delay(400);

	HAL_UART_Transmit(&huart5, (uint8_t*)"AT+CIPSEND\n\r", sizeof("AT+CIPSEND\n\r"), 100);
	HAL_UART_Transmit(&huart5, enter,sizeof(enter), 100);
	HAL_Delay(400);

	HAL_UART_Transmit(&huart5,Message, strlen(Message), 100);
	HAL_UART_Transmit(&huart5, enter,sizeof(enter), 100);
	HAL_UART_Transmit(&huart5, ctrlz, sizeof(ctrlz), 100);
	HAL_Delay(400);
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
  MX_UART4_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  __HAL_UART_ENABLE_IT(&huart4,UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart5,UART_IT_RXNE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(data_valid){
		  Result=Range_data[1];
		  Result=Result<<8;
		  Result=Result+Range_data[2];
		  Result/=10;
		  send_state=1;
		  if(send_state){
			  modem_package=0;
			  if(Result<50&&Result>22){
//			 	 AwakenClient_WriterServer(Result);
//				  HAL_UART_Transmit(&huart4, &ResultTxt,strlen(ResultTxt), 100);
//				  HAL_UART_Transmit(&huart4, (uint8_t*)"---\r\n", sizeof("---\r\n"), 100);
				  object=1;
//				  AwakenClient();
				  WriteServer(Result);
				  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);
				  modem_package=1;
				  send_state=0;
				  HAL_Delay(1000);
			  }else{
				  modem_package=0;
				  object=0;
				  //send_state=1;
				  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);
			  }
		  }else{
			  if(modem_package==1&&object==0){
				  send_state=1;
				  modem_package=0;
				  HAL_Delay(500);
			  }
		  }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_UART5;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin led_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
