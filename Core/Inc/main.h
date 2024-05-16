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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define PWMD_Pin GPIO_PIN_0
#define PWMD_GPIO_Port GPIOA
#define PWMB_Pin GPIO_PIN_1
#define PWMB_GPIO_Port GPIOA
#define PWMC_Pin GPIO_PIN_2
#define PWMC_GPIO_Port GPIOA
#define PWMA_Pin GPIO_PIN_3
#define PWMA_GPIO_Port GPIOA
#define E4A_Pin GPIO_PIN_5
#define E4A_GPIO_Port GPIOA
#define DAT_Pin GPIO_PIN_6
#define DAT_GPIO_Port GPIOA
#define CMD_Pin GPIO_PIN_7
#define CMD_GPIO_Port GPIOA
#define CS_Pin GPIO_PIN_0
#define CS_GPIO_Port GPIOB
#define CLK_Pin GPIO_PIN_1
#define CLK_GPIO_Port GPIOB
#define DIN1_Pin GPIO_PIN_2
#define DIN1_GPIO_Port GPIOB
#define DIN2_Pin GPIO_PIN_10
#define DIN2_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_12
#define BIN2_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_13
#define BIN1_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_14
#define AIN2_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_15
#define AIN1_GPIO_Port GPIOB
#define E1B_Pin GPIO_PIN_8
#define E1B_GPIO_Port GPIOA
#define E1A_Pin GPIO_PIN_9
#define E1A_GPIO_Port GPIOA
#define CIN2_Pin GPIO_PIN_11
#define CIN2_GPIO_Port GPIOA
#define CIN1_Pin GPIO_PIN_12
#define CIN1_GPIO_Port GPIOA
#define E4B_Pin GPIO_PIN_3
#define E4B_GPIO_Port GPIOB
#define E2A_Pin GPIO_PIN_4
#define E2A_GPIO_Port GPIOB
#define E2B_Pin GPIO_PIN_5
#define E2B_GPIO_Port GPIOB
#define E3B_Pin GPIO_PIN_6
#define E3B_GPIO_Port GPIOB
#define E3A_Pin GPIO_PIN_7
#define E3A_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
