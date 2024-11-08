/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f7xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B2_Pin GPIO_PIN_2
#define B2_GPIO_Port GPIOE
#define DP1_Pin GPIO_PIN_3
#define DP1_GPIO_Port GPIOE
#define Button_Pin GPIO_PIN_13
#define Button_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOF
#define C1_Pin GPIO_PIN_7
#define C1_GPIO_Port GPIOF
#define D1_Pin GPIO_PIN_9
#define D1_GPIO_Port GPIOF
#define A2_Pin GPIO_PIN_0
#define A2_GPIO_Port GPIOA
#define G2_Pin GPIO_PIN_0
#define G2_GPIO_Port GPIOB
#define F1_Pin GPIO_PIN_0
#define F1_GPIO_Port GPIOG
#define E1_Pin GPIO_PIN_1
#define E1_GPIO_Port GPIOG
#define DP2_Pin GPIO_PIN_12
#define DP2_GPIO_Port GPIOE
#define C2_Pin GPIO_PIN_15
#define C2_GPIO_Port GPIOE
#define D2_Pin GPIO_PIN_10
#define D2_GPIO_Port GPIOB
#define E2_Pin GPIO_PIN_11
#define E2_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_14
#define LED_RED_GPIO_Port GPIOB
#define Row1_Pin GPIO_PIN_8
#define Row1_GPIO_Port GPIOC
#define Row2_Pin GPIO_PIN_9
#define Row2_GPIO_Port GPIOC
#define Row3_Pin GPIO_PIN_10
#define Row3_GPIO_Port GPIOC
#define Row4_Pin GPIO_PIN_11
#define Row4_GPIO_Port GPIOC
#define A1_Pin GPIO_PIN_0
#define A1_GPIO_Port GPIOD
#define G1_Pin GPIO_PIN_1
#define G1_GPIO_Port GPIOD
#define Col1_Pin GPIO_PIN_4
#define Col1_GPIO_Port GPIOD
#define Col2_Pin GPIO_PIN_5
#define Col2_GPIO_Port GPIOD
#define Col3_Pin GPIO_PIN_6
#define Col3_GPIO_Port GPIOD
#define Col4_Pin GPIO_PIN_7
#define Col4_GPIO_Port GPIOD
#define LED_BLUE_Pin GPIO_PIN_7
#define LED_BLUE_GPIO_Port GPIOB
#define F2_Pin GPIO_PIN_0
#define F2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
