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
    for (int i = 0; i < 150; i++) {
      // printf("data[%d]: %c\r\n", i, rxBuffer[i]);
      rx_data_sim[i] = rxBuffer[i];
      if ((char)rxBuffer[i] == (char)SERIAL_NUMBER[5] && (char)rxBuffer[i + 1] == (char)SERIAL_NUMBER[6] && (char)rxBuffer[i + 2] == (char)SERIAL_NUMBER[7]) {
        //static int num_load = 0;
        // static int status;
        payLoadPin = (rxBuffer[i + 4] - 48);
        printf("-----------Receiver RELAY %d -----------\r\n", payLoadPin);
#if SIMCOM_MODEL == a7672
        if (rxBuffer[(i + 29)] == 49 && isPBDONE == true)
#elif SIMCOM_MODEL == a7670
        if (rxBuffer[(i + 31)] == 49 && isPBDONE == true)
#endif
        {
          //payLoadPin = (rxBuffer[i + 4] - 48);
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
int connectMQTT(void) {
  /*enable_mqtt_on_gsm_modem();*/
  sprintf(AT_COMMAND, "+CMQTTACCQ: 0,\"%s\",0\r\n", MQTT_CLIENT_ID);
  sendingToSimcomA76xx("AT+CMQTTACCQ?\r\n");
  if (strstr((char *)rx_data_sim, AT_COMMAND) != NULL) {
    printf("-----------------Had acquired------------------\n");
    //     AT_Acquier_MQTT = true;
    return 1;
  }
  sendingToSimcomA76xx("AT+CMQTTSTART\r\n");
  HAL_Delay(200);
  sprintf(AT_COMMAND, "AT+CMQTTACCQ=0,\"%s\",0\r\n", MQTT_CLIENT_ID);
  sendingToSimcomA76xx(AT_COMMAND);
  HAL_Delay(200);
  sprintf(AT_COMMAND, "AT+CMQTTCONNECT=0,\"%s:%d\",500,1,\"%s\",\"%s\"\r\n", MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_PASS);
  sendingToSimcomA76xx(AT_COMMAND);
  HAL_Delay(200);
  for (int i = 1; i <= NUMBER_LOADS; i++) {
    isConnectMQTT = 0;
    previousTick = HAL_GetTick();
    sprintf(AT_COMMAND, "AT+CMQTTSUBTOPIC=0,%d,1\r\n", (strlen(MQTT_TOPIC_ACTUATOR_CONTROL) + 1));
    sendingToSimcomA76xx(AT_COMMAND);
    HAL_Delay(200);
    sprintf(AT_COMMAND, "%s%d\r\n", MQTT_TOPIC_ACTUATOR_CONTROL, i);
    sendingToSimcomA76xx(AT_COMMAND);
    HAL_Delay(200);
    sendingToSimcomA76xx("AT+CMQTTSUB=0\r\n");
    HAL_Delay(200);
    while (isConnectMQTT == 0 && previousTick + timeOutConnectMQTT > HAL_GetTick()) {

      if (strstr((char *)rx_data_sim, "CMQTTSUB: 0,0")) {
        // MX_IWDG_Init();
        isConnectMQTT = 1;
      }
    }
    if (isConnectMQTT == 0) {
      NVIC_SystemReset();
      ;
    }
  }
  if (isConnectMQTT == 1) {
    HAL_GPIO_WritePin(GPIOB, LED_STATUS_Pin, GPIO_PIN_SET);
  }
  return isConnectMQTT;
}
void create_JSON(void) {
  cJSON *json = cJSON_CreateObject();
  for (int i = 1; i < NUMBER_LOADS + 1; i++) {
    int statusOfLoad;
    statusOfLoad = HAL_GPIO_ReadPin(GPIO_LOAD_PORT[i - 1], GPIO_LOAD_PIN[i - 1]);
    char payload1[2];
    sprintf(payload1, "%d", i);
    cJSON_AddNumberToObject(json, payload1, statusOfLoad);
  }
  Data_Percentage_pin = Level_Pin();
  rssi = read_signal_quality();
  cJSON_AddNumberToObject(json, "_gsm_signal_strength", rssi);
  cJSON_AddNumberToObject(json, "_battery_level", Data_Percentage_pin);
  char *json_string = cJSON_Print(json);
  if (json_string == NULL) {
    printf("Lỗi tạo chuỗi JSON\n");
    cJSON_Delete(json);
    return;
  }
  sprintf(array_json, "%s", json_string);
  // decompress memory
  free(json_string);
  cJSON_Delete(json);
}
