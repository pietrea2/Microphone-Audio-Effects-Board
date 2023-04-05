/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define AUDIO0_Pin GPIO_PIN_0
#define AUDIO0_GPIO_Port GPIOF
#define AUDIO1_Pin GPIO_PIN_1
#define AUDIO1_GPIO_Port GPIOF
#define AUDIO2_Pin GPIO_PIN_2
#define AUDIO2_GPIO_Port GPIOF
#define AUDIO3_Pin GPIO_PIN_3
#define AUDIO3_GPIO_Port GPIOF
#define AUDIO4_Pin GPIO_PIN_4
#define AUDIO4_GPIO_Port GPIOF
#define AUDIO5_Pin GPIO_PIN_5
#define AUDIO5_GPIO_Port GPIOF
#define AUDIO6_Pin GPIO_PIN_6
#define AUDIO6_GPIO_Port GPIOF
#define AUDIO7_Pin GPIO_PIN_7
#define AUDIO7_GPIO_Port GPIOF
#define AUDIO8_Pin GPIO_PIN_8
#define AUDIO8_GPIO_Port GPIOF
#define AUDIO9_Pin GPIO_PIN_9
#define AUDIO9_GPIO_Port GPIOF
#define AUDIO10_Pin GPIO_PIN_10
#define AUDIO10_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define AUDIO11_Pin GPIO_PIN_11
#define AUDIO11_GPIO_Port GPIOF
#define AUDIO12_Pin GPIO_PIN_12
#define AUDIO12_GPIO_Port GPIOF
#define AUDIO13_Pin GPIO_PIN_13
#define AUDIO13_GPIO_Port GPIOF
#define AUDIO14_Pin GPIO_PIN_14
#define AUDIO14_GPIO_Port GPIOF
#define AUDIO15_Pin GPIO_PIN_15
#define AUDIO15_GPIO_Port GPIOF
#define SNSS_Pin GPIO_PIN_11
#define SNSS_GPIO_Port GPIOE
#define DE1_AUDIO_WR_Pin GPIO_PIN_10
#define DE1_AUDIO_WR_GPIO_Port GPIOB
#define DE1_AUDIO_READY_Pin GPIO_PIN_11
#define DE1_AUDIO_READY_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/*
#define SPI_SNSS_Pin GPIO_PIN_13
#define SPI_SNSS_GPIO_Port GPIOF

//TESTING GPIO BUS
#define TEST_GPIO_1 GPIO_PIN_7
#define TEST_GPIO_2 GPIO_PIN_6
#define TEST_GPIO_3 GPIO_PIN_5
#define TEST_GPIO_4 GPIO_PIN_4

#define AUDIO_GPIO_0 GPIO_PIN_7
#define AUDIO_GPIO_1 GPIO_PIN_6
#define AUDIO_GPIO_2 GPIO_PIN_5
#define AUDIO_GPIO_3 GPIO_PIN_4
#define AUDIO_GPIO_PINS_0_3_PORT GPIOD

#define AUDIO_GPIO_4 GPIO_PIN_2
#define AUDIO_GPIO_5 GPIO_PIN_4
#define AUDIO_GPIO_6 GPIO_PIN_5
#define AUDIO_GPIO_7 GPIO_PIN_6

//NEW
#define AUDIO_GPIO_8 GPIO_PIN_3
#define AUDIO_GPIO_PINS_4_8_PORT GPIOE

#define AUDIO_GPIO_9 GPIO_PIN_8
#define AUDIO_GPIO_10 GPIO_PIN_7
#define AUDIO_GPIO_11 GPIO_PIN_9
#define AUDIO_GPIO_PINS_9_11_PORT GPIOF

#define AUDIO_GPIO_12 GPIO_PIN_1
#define AUDIO_GPIO_PIN_12_PORT GPIOG

#define AUDIO_GPIO_13 GPIO_PIN_0
#define AUDIO_GPIO_14 GPIO_PIN_1
#define AUDIO_GPIO_PINS_13_14_PORT GPIOD

#define AUDIO_GPIO_15 GPIO_PIN_0
#define AUDIO_GPIO_PIN_15_PORT GPIOG

//DE1 signals
#define AUDIO_READY GPIO_PIN_15 //FPGA i ready to recieve data
#define AUDIO_READY_GPIO_PORT GPIOE

#define AUDIO_WRITE GPIO_PIN_10
#define AUDIO_ENABLE GPIO_PIN_11
#define AUDIO_WRITE_AND_ENABLE_GPIO_PORT GPIOB

#define AUDIO_CLEAR_BUF GPIO_PIN_14
#define AUDIO_CLEAR_BUF_GPIO_PORT GPIOE
*/




/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
