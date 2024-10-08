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
#include "stm32f1xx_hal.h"

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
void Level_Pin (void);
void aray (void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ON_OFF_PWM_Pin GPIO_PIN_3
#define ON_OFF_PWM_GPIO_Port GPIOA
#define PAYLOAD_4_Pin GPIO_PIN_4
#define PAYLOAD_4_GPIO_Port GPIOC
#define PAYLOAD_3_Pin GPIO_PIN_5
#define PAYLOAD_3_GPIO_Port GPIOC
#define PAYLOAD_2_Pin GPIO_PIN_0
#define PAYLOAD_2_GPIO_Port GPIOB
#define PAYLOAD_1_Pin GPIO_PIN_1
#define PAYLOAD_1_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_8
#define LED_GPIO_Port GPIOA
#define A76XX_PWRKEY_Pin GPIO_PIN_11
#define A76XX_PWRKEY_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
