/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <string.h>
//#include "usb_host.h"
//#include "fatfs.h"
//#include "usb_conf.h"
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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t get_spi_byte(void)
{
	uint8_t data;
	HAL_SPI_Receive(&hspi1, &data, 1, 100);
	while(hspi1.State == HAL_SPI_STATE_BUSY);
	return data;
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
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  int strindex = 0;
  char strbuf[6000];
  char strbuf2[100];

  //set CS initially to:
  HAL_GPIO_WritePin(SPI_SNSS_GPIO_Port, SPI_SNSS_Pin, GPIO_PIN_RESET); // NSS1 Default Set



  //DE1 GPIO data to send
  //HAL_GPIO_WritePin(AUDIO_READY_GPIO_PORT, AUDIO_READY, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AUDIO_CLEAR_BUF_GPIO_PORT, AUDIO_CLEAR_BUF, GPIO_PIN_RESET); //set to 0 before sending every NUMVALS


  HAL_GPIO_WritePin(AUDIO_GPIO_PINS_0_3_PORT, AUDIO_GPIO_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AUDIO_GPIO_PINS_0_3_PORT, AUDIO_GPIO_1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AUDIO_GPIO_PINS_0_3_PORT, AUDIO_GPIO_2, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AUDIO_GPIO_PINS_0_3_PORT, AUDIO_GPIO_3, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(AUDIO_GPIO_PINS_4_8_PORT, AUDIO_GPIO_4, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AUDIO_GPIO_PINS_4_8_PORT, AUDIO_GPIO_5, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AUDIO_GPIO_PINS_4_8_PORT, AUDIO_GPIO_6, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AUDIO_GPIO_PINS_4_8_PORT, AUDIO_GPIO_7, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(AUDIO_GPIO_PINS_4_8_PORT, AUDIO_GPIO_8, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AUDIO_GPIO_PINS_9_11_PORT, AUDIO_GPIO_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AUDIO_GPIO_PINS_9_11_PORT, AUDIO_GPIO_10, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AUDIO_GPIO_PINS_9_11_PORT, AUDIO_GPIO_11, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(AUDIO_GPIO_PIN_12_PORT, AUDIO_GPIO_12, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AUDIO_GPIO_PINS_13_14_PORT, AUDIO_GPIO_13, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AUDIO_GPIO_PINS_13_14_PORT, AUDIO_GPIO_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AUDIO_GPIO_PIN_15_PORT, AUDIO_GPIO_15, GPIO_PIN_RESET);


  HAL_GPIO_WritePin(AUDIO_WRITE_AND_ENABLE_GPIO_PORT, AUDIO_WRITE, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AUDIO_WRITE_AND_ENABLE_GPIO_PORT, AUDIO_ENABLE, GPIO_PIN_SET);


  //128 samples per buffer



#define NUM_VALS 128
  int vindex = 0;
  uint16_t values[NUM_VALS];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */





	uint16_t value = 0;


	HAL_GPIO_WritePin(SPI_SNSS_GPIO_Port, SPI_SNSS_Pin, GPIO_PIN_RESET); // NSS1 low
	//GPIO_PinState bit0 = HAL_GPIO_ReadPin(SPI_SNSS_GPIO_Port, SPI_SNSS_Pin);
    //if(audio_ready_bit == GPIO_PIN_RESET){HAL_UART_Transmit(&huart3, (uint8_t*)"\nCS = 0:\n", strlen("\nBit 1 = 0:\n"), 100);}
	//	sprintf(strbuf2, "\nCS = 0:\n", bit0);
	//	HAL_UART_Transmit(&huart3, (uint8_t*)strbuf2, strlen(strbuf2), 100);


	value |= get_spi_byte();
	value |= get_spi_byte() << 8;
	HAL_GPIO_WritePin(SPI_SNSS_GPIO_Port, SPI_SNSS_Pin, GPIO_PIN_SET); // NSS1 low

	//bit0 = HAL_GPIO_ReadPin(SPI_SNSS_GPIO_Port, SPI_SNSS_Pin);
	//if(audio_ready_bit == GPIO_PIN_RESET){HAL_UART_Transmit(&huart3, (uint8_t*)"\nCS = 0:\n", strlen("\nBit 1 = 0:\n"), 100);}
	//sprintf(strbuf2, "\nCS = 0:\n", bit0);
	//HAL_UART_Transmit(&huart3, (uint8_t*)strbuf2, strlen(strbuf2), 100);


	if (vindex >= NUM_VALS)
	{
		//HAL_UART_Transmit(&huart3, (uint8_t*)"\n\nBegin:\n", strlen("\n\nBegin:\n"), 100);
		//CDC_Transmit_FS((uint8_t*)"\n\nBegin:\n", strlen("\n\nBegin:\n"));

		vindex = 0; strindex = 0;
		//for (int i = 0; i < NUM_VALS; ++i)
		//	strindex += sprintf(strbuf + strindex, "%d, ", values[i]);

		//HAL_UART_Transmit(&huart3, (uint8_t*)strbuf, strindex, 100);
		//CDC_Transmit_FS((uint8_t*)strbuf, strindex);

		//write buffer to GPIO pins


		HAL_GPIO_WritePin(AUDIO_CLEAR_BUF_GPIO_PORT, AUDIO_CLEAR_BUF, GPIO_PIN_SET); //set to 0 before sending every NUMVALS
		HAL_Delay(1000);
		HAL_GPIO_WritePin(AUDIO_CLEAR_BUF_GPIO_PORT, AUDIO_CLEAR_BUF, GPIO_PIN_RESET);
		while(HAL_GPIO_ReadPin(AUDIO_READY_GPIO_PORT, AUDIO_READY) != GPIO_PIN_SET);



		for (int i = 0; i < NUM_VALS; ++i){

			  HAL_GPIO_WritePin(AUDIO_WRITE_AND_ENABLE_GPIO_PORT, AUDIO_WRITE, GPIO_PIN_SET);

			  HAL_GPIO_WritePin(AUDIO_GPIO_PINS_0_3_PORT, AUDIO_GPIO_0, values[i] & 0x01 );
			  HAL_GPIO_WritePin(AUDIO_GPIO_PINS_0_3_PORT, AUDIO_GPIO_1, (values[i] & 0x02) >> 1 );
			  HAL_GPIO_WritePin(AUDIO_GPIO_PINS_0_3_PORT, AUDIO_GPIO_2, (values[i] & 0x04) >> 2);
			  HAL_GPIO_WritePin(AUDIO_GPIO_PINS_0_3_PORT, AUDIO_GPIO_3, (values[i] & 0x08) >> 3);

			  HAL_GPIO_WritePin(AUDIO_GPIO_PINS_4_8_PORT, AUDIO_GPIO_4, (values[i] & 0x10) >> 4);
			  HAL_GPIO_WritePin(AUDIO_GPIO_PINS_4_8_PORT, AUDIO_GPIO_5, (values[i] & 0x20) >> 5);
			  HAL_GPIO_WritePin(AUDIO_GPIO_PINS_4_8_PORT, AUDIO_GPIO_6, (values[i] & 0x40) >> 6);
			  HAL_GPIO_WritePin(AUDIO_GPIO_PINS_4_8_PORT, AUDIO_GPIO_7, (values[i] & 0x80) >> 7);

			  HAL_GPIO_WritePin(AUDIO_GPIO_PINS_4_8_PORT, AUDIO_GPIO_8, (values[i] & 0x100) >> 8);
			  HAL_GPIO_WritePin(AUDIO_GPIO_PINS_9_11_PORT, AUDIO_GPIO_9, (values[i] & 0x200) >> 9);
			  HAL_GPIO_WritePin(AUDIO_GPIO_PINS_9_11_PORT, AUDIO_GPIO_10, (values[i] & 0x400) >> 10);
			  HAL_GPIO_WritePin(AUDIO_GPIO_PINS_9_11_PORT, AUDIO_GPIO_11, (values[i] & 0x800) >> 11);

			  HAL_GPIO_WritePin(AUDIO_GPIO_PIN_12_PORT, AUDIO_GPIO_12, (values[i] & 0x1000) >> 12);
			  HAL_GPIO_WritePin(AUDIO_GPIO_PINS_13_14_PORT, AUDIO_GPIO_13, (values[i] & 0x2000) >> 13);
			  HAL_GPIO_WritePin(AUDIO_GPIO_PINS_13_14_PORT, AUDIO_GPIO_14, (values[i] & 0x4000) >> 14);
			  HAL_GPIO_WritePin(AUDIO_GPIO_PIN_15_PORT, AUDIO_GPIO_15, (values[i] & 0x8000) >> 15);

			  HAL_Delay(1000);
			  HAL_GPIO_WritePin(AUDIO_WRITE_AND_ENABLE_GPIO_PORT, AUDIO_WRITE, GPIO_PIN_RESET);
			  //HAL_Delay(50);
		}


	}
	else { values[vindex++] = value; }




	//GPIO_PinState audio_ready_bit = HAL_GPIO_ReadPin(AUDIO_READY_GPIO_PORT, AUDIO_READY);

	//GPIO_PinState bit0 = HAL_GPIO_ReadPin(AUDIO_GPIO_PINS_0_3_PORT, AUDIO_GPIO_0);



	/*
	GPIO_PinState bit3 = HAL_GPIO_ReadPin(TEST_GPIO_Port_1, GPIO_PIN_5);
	GPIO_PinState bit4 = HAL_GPIO_ReadPin(TEST_GPIO_Port_1, GPIO_PIN_4);


	GPIO_PinState bit5 = HAL_GPIO_ReadPin(TEST_GPIO_Port_2, GPIO_PIN_2);
	GPIO_PinState bit6 = HAL_GPIO_ReadPin(TEST_GPIO_Port_2, GPIO_PIN_4);
	GPIO_PinState bit7 = HAL_GPIO_ReadPin(TEST_GPIO_Port_2, GPIO_PIN_5);
	GPIO_PinState bit8 = HAL_GPIO_ReadPin(TEST_GPIO_Port_2, GPIO_PIN_6);
	*/

	//if(audio_ready_bit == GPIO_PIN_RESET){HAL_UART_Transmit(&huart3, (uint8_t*)"\nAudio Ready = 0:\n", strlen("\nBit 1 = 0:\n"), 100);}
	//if(bit0 == GPIO_PIN_RESET){HAL_UART_Transmit(&huart3, (uint8_t*)"\nBit 0 = 0:\n", strlen("\nBit 1 = 0:\n"), 100);}
	//HAL_Delay(0);





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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 5;
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
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|GPIO_PIN_10|GPIO_PIN_11|LD3_Pin
                          |LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0|GPIO_PIN_1|USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5
                           PE6 PE14 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PF7 PF8 PF9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PF13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD4 PD5
                           PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
