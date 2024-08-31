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
#include "stdbool.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
float Level_Pin (void);
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define a7672 1
#define a7670 2

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
extern int connectMQTT(void);
extern void sendingToSimcomA76xx(char *cmd);
extern void create_JSON(void);
extern float fn_check_signal_simcom(void);
extern int acquire_gsm_mqtt_client(void);
extern int enable_mqtt_on_gsm_modem(void);
extern int connect_mqtt_server_by_gsm(void);
extern int subscribe_mqtt_via_gsm(void);
extern int publish_mqtt_via_gsm(void);
extern int read_signal_quality(void);
extern int stop_mqtt_via_gsm(void);
extern int check_error_mqtt_via_gsm(void);
extern int update_status(void);
extern void restart_stm32(void);
extern int init_cricket(void);
extern int event_wait_function(void);
extern int check_active_payload(void);
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

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
#define LED_STATUS_Pin GPIO_PIN_12
#define LED_STATUS_GPIO_Port GPIOB
#define A76XX_PWRKEY_Pin GPIO_PIN_11
#define A76XX_PWRKEY_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
extern char rxBuffer[150];
extern char rx_data_sim[150];
extern char AT_COMMAND[100];
extern int isPBDONE;
extern int isATOK;
extern int onReay;
extern int isConnectMQTT;
extern int previousTick;
extern int timeOutConnectMQTT;
extern int payLoadPin,payLoadStatus;
extern char array_json[100];
extern float Data_Percentage_pin;
extern float SignalStrength;
extern int rssi;
extern int isConnectSimcomA76xx;
extern bool fn_Enable_MQTT;
extern bool fn_Connect_MQTT;
extern bool fn_CheckSim;
extern bool fn_Subcribe_MQTT;
extern bool fn_Publish_MQTT;
extern bool fn_Acquier_MQTT;
extern bool fn_update_status;
extern GPIO_TypeDef* GPIO_LOAD_PORT[4];
extern unsigned int GPIO_LOAD_PIN[4];



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
