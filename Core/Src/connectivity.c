/*
 * connectivity.c
 *
 *  Created on: Aug 27, 2024
 *      Author: thuanphat7
 */
#include "cJSON.h"
#include "config.h"
#include "main.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <math.h>
extern UART_HandleTypeDef huart1;
char array_json[100];
// float Percentage_battery;

void sendingToSimcomA76xx(char *cmd) {
  printf("STM32 Write: %s", cmd);
  HAL_UART_Transmit(&huart1, (uint8_t *)cmd, strlen(cmd), 1000);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart->Instance == USART1) {
    printf("\r\nSIMCOM Response:");
    printf(rxBuffer);
    static int times;
    times=strlen(rxBuffer);
    for (int i = 0; i < times; i++) {
      // printf("data[%d]: %c\r\n", i, rxBuffer[i]);
      rx_data_sim[i] = rxBuffer[i];
      if ((char)rxBuffer[i] == (char)SERIAL_NUMBER[5] && (char)rxBuffer[i + 1] == (char)SERIAL_NUMBER[6] && (char)rxBuffer[i + 2] == (char)SERIAL_NUMBER[7]) {
        //static int num_load = 0;
        // static int status;
        payLoadPin = (rxBuffer[i + 4] - 48);
#if SIMCOM_MODEL == a7672
        if (rxBuffer[(i + 29)] == 49 && isPBDONE == true)
#elif SIMCOM_MODEL == a7670
        if (rxBuffer[(i + 31)] == 49 && isPBDONE == true)
#endif
        {
          //payLoadPin = (rxBuffer[i + 4] - 48);
          printf("-----------ON RELAY %d -----------\r\n", payLoadPin);
          HAL_GPIO_WritePin(GPIO_LOAD_PORT[payLoadPin - 1], GPIO_LOAD_PIN[payLoadPin - 1], 1);
          onReay++;
          if (onReay >= NUMBER_LOADS) {
            onReay = NUMBER_LOADS;
          }
          HAL_GPIO_WritePin(ON_OFF_PWM_GPIO_Port, ON_OFF_PWM_Pin, 0);
        }
#if SIMCOM_MODEL == a7672
        if (rxBuffer[(i + 29)] == 48 && isPBDONE == true)
#elif SIMCOM_MODEL == a7670
        if (rxBuffer[(i + 31)] == 48 && isPBDONE == true)
#endif
        {
          //payLoadPin = (rxBuffer[i + 4] - 48);
        printf("-----------OFF RELAY %d -----------\r\n", payLoadPin);
          HAL_GPIO_WritePin(GPIO_LOAD_PORT[payLoadPin - 1], GPIO_LOAD_PIN[payLoadPin - 1], 0);
          --onReay;
          if (onReay <= 0) {
            onReay = 0;
            HAL_GPIO_WritePin(ON_OFF_PWM_GPIO_Port, ON_OFF_PWM_Pin, 1);
          }
        }
      }
    }
    if ((strstr((char *)rxBuffer, "+CMQTTCONNLOST") != NULL) && isPBDONE == true) {
      printf("-----------------Client Disconnect passively!------------------\n");
     check_error_mqtt_via_gsm();
    }
    //onReay=check_active_payload();
    memset(rxBuffer, '\0', 150);
  }
  HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t *)rxBuffer, 150);
}
void create_JSON(void) {
  cJSON *json = cJSON_CreateObject();
  for (int i = 1; i < NUMBER_LOADS + 1; i++) {
    int statusOfLoad;
    statusOfLoad = HAL_GPIO_ReadPin(GPIO_LOAD_PORT[i - 1], GPIO_LOAD_PIN[i - 1]);
    static char payload1[2];
    sprintf(payload1, "%d", i);
    cJSON_AddNumberToObject(json, payload1, statusOfLoad);
  }
  Data_Percentage_pin = Level_Pin();
  rssi = read_signal_quality();
  cJSON_AddNumberToObject(json, "_gsm_signal_strength", rssi);
  cJSON_AddNumberToObject(json, "_battery_level", Data_Percentage_pin);
  char *json_string = cJSON_Print(json);
  if (json_string == NULL) {
    printf("New create error JSON\n");
    cJSON_Delete(json);
    return;
  }
  sprintf(array_json, "%s", json_string);
  // decompress memory
  free(json_string);
  cJSON_Delete(json);
}
