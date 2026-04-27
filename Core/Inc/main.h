/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_GREEN_R_Pin GPIO_PIN_13
#define LED_GREEN_R_GPIO_Port GPIOC
#define LED_RED_R_Pin GPIO_PIN_14
#define LED_RED_R_GPIO_Port GPIOC
#define MOTOR4_Pin GPIO_PIN_5
#define MOTOR4_GPIO_Port GPIOA
#define LED_GREEN_L_Pin GPIO_PIN_6
#define LED_GREEN_L_GPIO_Port GPIOA
#define LED_RED_L_Pin GPIO_PIN_7
#define LED_RED_L_GPIO_Port GPIOA
#define MODULE_POWER_Pin_Pin GPIO_PIN_0
#define MODULE_POWER_Pin_GPIO_Port GPIOB
#define MODULE_DETECT_Pin_Pin GPIO_PIN_1
#define MODULE_DETECT_Pin_GPIO_Port GPIOB
#define MOTOR3_Pin GPIO_PIN_10
#define MOTOR3_GPIO_Port GPIOB
#define LED_BLUE_L_Pin GPIO_PIN_12
#define LED_BLUE_L_GPIO_Port GPIOB
#define MODULE_SHARED_SCL_Pin_Pin GPIO_PIN_8
#define MODULE_SHARED_SCL_Pin_GPIO_Port GPIOA
#define MODULE_SHARED_DATA_Pin_Pin GPIO_PIN_4
#define MODULE_SHARED_DATA_Pin_GPIO_Port GPIOB
#define MOTOR2_Pin GPIO_PIN_6
#define MOTOR2_GPIO_Port GPIOB
#define MOTOR1_Pin GPIO_PIN_7
#define MOTOR1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
