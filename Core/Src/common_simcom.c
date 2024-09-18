/*
 * common_simcom.c
 *
 *  Created on: Aug 28, 2024
 *      Author: thuanphat7
 */
#include "cJSON.h"
#include "config.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <main.h>
#include <stdbool.h>

bool AT_Connect_MQTT = false;
bool AT_Acquier_MQTT = false;
bool AT_Subcribe_Topic_MQTT = false;
bool AT_Subcribe_MQTT = false;
bool AT_Topic_Puplish_MQTT = false;
bool AT_Data_Puplish_MQTT = false;
bool AT_Puplish_MQTT = false;
bool AT_Check_Dis_MQTT = false;
bool AT_Disconnect_MQTT = false;
bool AT_Rel_MQTT = false;
bool AT_Stop_MQTT = false;
bool inital_check = false;
uint16_t count_errors = 0;
int timeout_pb_done = 40000;

int read_signal_quality(void) {
  sendingToSimcomA76xx("AT+CSQ\r\n");
  HAL_Delay(200);
  SignalStrength = (rx_data_sim[8] - 48) * 10 + (rx_data_sim[9] - 48);
  if (SignalStrength >= 31) {
    rssi = -51;
  } else if (SignalStrength <= 0) {
    rssi = -113;
  } else
    rssi = (SignalStrength * 2 - 113);

  return rssi;
}
float fn_check_signal_simcom(void) {
  printf("-----------------fn_check_signal_simcom------------------\n");
  sendingToSimcomA76xx("ATE0\r\n");
  HAL_Delay(200);
  sendingToSimcomA76xx("AT+CSQ\r\n");
  HAL_Delay(200);
  SignalStrength = (rx_data_sim[8] - 48) * 10 + (rx_data_sim[9] - 48);
  if (SignalStrength >= 31) {
    rssi = -51;
  } else
    rssi = (SignalStrength * 2 - 113);
  isConnectSimcomA76xx = 1;
  HAL_Delay(200);
  sendingToSimcomA76xx("AT+CPIN?\r\n");
  HAL_Delay(200);
  if (strstr((char *)rx_data_sim, "+CPIN: READY")) {
    printf("-----------------SIM OK !------------------\n");
  } else
    return 0;
  //  sendingToSimcomA76xx("AT+CREG=2\r\n");
  //  HAL_Delay(5000);
  HAL_Delay(200);
  sendingToSimcomA76xx("AT+CREG?\r\n");
  HAL_Delay(200);
  if (strstr((char *)rx_data_sim, "+CREG: 0,1") || strstr((char *)rx_data_sim, "+CREG: 0,6") || strstr((char *)rx_data_sim, "+CREG: 2,6")) {
    printf("-----------------Network registration OK!------------------\n");
  } else
    return 0;

  sendingToSimcomA76xx("ATI\r\n");
  HAL_Delay(200);
  sendingToSimcomA76xx("AT+CICCID\r\n");
  HAL_Delay(200);
  sendingToSimcomA76xx("AT+CGREG?\r\n");
  HAL_Delay(200);

  if (strstr((char *)rx_data_sim, "+CGREG: 0,1")) {
    printf("-----------------Network registration OK!------------------\n");
  } else
    return 0;
  return 1;
}
int enable_mqtt_on_gsm_modem(void) {
  sendingToSimcomA76xx("AT+CMQTTSTART\r\n");
  HAL_Delay(400);
  if ((strstr((char *)rx_data_sim, "+CMQTTSTART: 0") != NULL) || (strstr((char *)rx_data_sim, "ERROR") != NULL)) {
    printf("-----------------Service have started "
           "successfully------------------\n");
    return 1;
  } else {
    printf("----------------- Start MQTT service fail------------------\n");
    return 0;
  }
  return 0;
}
int acquire_gsm_mqtt_client(void) {
  printf("-----------------acquire_gsm_mqtt_client------------------\n");
  sprintf(AT_COMMAND, "+CMQTTACCQ: 0,\"%s\",0\r\n", MQTT_CLIENT_ID);
  sendingToSimcomA76xx("AT+CMQTTACCQ?\r\n");
  HAL_Delay(400);
  if (strstr((char *)rx_data_sim, AT_COMMAND) != NULL) {
    printf("-----------------Had acquired------------------\n");
    return 1;
  } else {
    printf("-----------------Haven't got acquier yet------------------\n");
    AT_Acquier_MQTT = false;
  }
  if (AT_Acquier_MQTT == false) {
    sprintf(AT_COMMAND, "AT+CMQTTACCQ=0,\"%s\",0\r\n", MQTT_CLIENT_ID);
    sendingToSimcomA76xx(AT_COMMAND);
    HAL_Delay(200);
    sprintf(AT_COMMAND, "+CMQTTACCQ: 0,\"%s\",0", MQTT_CLIENT_ID);
    HAL_Delay(200);
    if (strstr((char *)rx_data_sim, "OK") != NULL) {
      printf("-----------------Acquier Successfully------------------\n");
      AT_Acquier_MQTT = true;
      return 1;
    } else {
      printf("-----------------Acquier Fail------------------\n");
    }
  }
  return 0;
}
int connect_mqtt_server_by_gsm(void) {
  sprintf(AT_COMMAND, "+CMQTTCONNECT: 0,\"%s:%d\",20,1,\"%s\",\"%s\"\r\n", MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_PASS);
  HAL_Delay(200);
  sendingToSimcomA76xx("AT+CMQTTCONNECT?\r\n");
  HAL_Delay(200);
  if (strstr((char *)rx_data_sim, AT_COMMAND) != NULL) {
    printf("-----------------Connected------------------\n");
    AT_Connect_MQTT = true;
    return 1;
  } else {
    printf("-----------------Not connect yet !------------------\n");
    AT_Connect_MQTT = false;
  }
  if (AT_Connect_MQTT == false) {
    sprintf(AT_COMMAND, "AT+CMQTTCONNECT=0,\"%s:%d\",20,1,\"%s\",\"%s\"\r\n", MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_PASS);
    sendingToSimcomA76xx(AT_COMMAND);
    HAL_Delay(500);
    if (strstr((char *)rx_data_sim, "+CMQTTCONNECT: 0,0") != NULL) {
      printf("-----------------Connected MQTT Success------------------\n");
      return 1;
    } else {
      printf("-----------------Connect fail------------------\n");
    }
  }
  return 0;
}
int subscribe_mqtt_via_gsm(void) {
  for (int i = 1; i < NUMBER_LOADS + 1; i++) {
    sprintf(AT_COMMAND, "%s/snac/%s/%d", FARM, SERIAL_NUMBER, i);
    sprintf(AT_COMMAND, "AT+CMQTTSUBTOPIC=0,%d,1\r\n", (int)strlen(AT_COMMAND));
    sendingToSimcomA76xx(AT_COMMAND);
    HAL_Delay(500);
    sprintf(AT_COMMAND, "%s/snac/%s/%d", FARM, SERIAL_NUMBER, i);
    sendingToSimcomA76xx(AT_COMMAND);
    HAL_Delay(500);
    if (strstr((char *)rx_data_sim, "OK") != NULL) {
      printf("-----------------Subscribe Topic Success------------------\n");
      AT_Subcribe_Topic_MQTT = true;
    } else {
      printf("-----------------Subscribe Topic Fail------------------\n");
      AT_Subcribe_Topic_MQTT = false;
    }
    if (AT_Subcribe_Topic_MQTT == true) {
      sendingToSimcomA76xx("AT+CMQTTSUB=0\r\n");
      HAL_Delay(500);
      if (strstr((char *)rx_data_sim, "+CMQTTSUB: 0,0") != NULL) {
        printf("-----------------Subscribe Successed !------------------\n");
        AT_Subcribe_MQTT = true;
      } else {
        printf("-----------------Subscribe Fail !------------------\n");
        AT_Subcribe_MQTT = false;
        return 0;
      }
    }
  }
  return 1;
}
int publish_mqtt_via_gsm(void) {
  //  is used to input the topic of a publish message
  create_JSON();
  sprintf(AT_COMMAND, "AT+CMQTTTOPIC=0,%d\r\n", strlen(MQTT_TOPIC_ACTUATOR_STATUS));
  sendingToSimcomA76xx(AT_COMMAND);
  HAL_Delay(200);
  sprintf(AT_COMMAND, "%s\r\n", MQTT_TOPIC_ACTUATOR_STATUS);
  sendingToSimcomA76xx(AT_COMMAND);
  HAL_Delay(200);
  if (strstr((char *)rx_data_sim, "OK") != NULL) {
    printf("----------------- Sent input the topic of a publish message success ! ------------------\n");
    AT_Topic_Puplish_MQTT = true;
  } else {
    printf("----------------- Sent input the topic of a publish message fail !------------------\n");
    AT_Topic_Puplish_MQTT = false;
  }
  if (AT_Topic_Puplish_MQTT) {
    // is used to input the message body of a publish message.
    int lengthOfInformPayload = strlen(array_json);
    sprintf(AT_COMMAND, "AT+CMQTTPAYLOAD=0,%d\r\n", lengthOfInformPayload);
    sendingToSimcomA76xx(AT_COMMAND);
    HAL_Delay(200);
    sendingToSimcomA76xx(array_json);
    HAL_Delay(200);
    if (strstr((char *)rx_data_sim, "OK") != NULL) {
      printf("----------------- Sent input the message body of a publish message ! ------------------\n");
      AT_Data_Puplish_MQTT = true;
    } else {
      printf("----------------- Sent input the message body of a publish fail! ------------------\n");
      AT_Data_Puplish_MQTT = false;
    }
    if (AT_Data_Puplish_MQTT) {
      sendingToSimcomA76xx("AT+CMQTTPUB=0,1,60\r\n");
      HAL_Delay(200);
      if (strstr((char *)rx_data_sim, "+CMQTTPUB: 0,0") != NULL) {
        printf("-----------------Publish Success !------------------\n");
        AT_Puplish_MQTT = true;
        return 1;
      } else {
        printf("-----------------Publish fail !------------------\n");
        AT_Puplish_MQTT = false;
      }
    }
  }
  return 0;
}
int check_error_mqtt_via_gsm(void) {
  fn_Enable_MQTT = false;
  fn_Connect_MQTT = false;
  fn_CheckSim = false;
  fn_Acquier_MQTT = false;
  if (!fn_CheckSim) {
    fn_CheckSim = fn_check_signal_simcom();
  } else {
    return 0;
  }

  if (fn_CheckSim) {
    fn_Enable_MQTT = enable_mqtt_on_gsm_modem();
  } else {
    return 0;
  }

  if (fn_Enable_MQTT) {
    fn_Acquier_MQTT = acquire_gsm_mqtt_client();
  } else {
    return 0;
  }
  if (fn_Acquier_MQTT) {
    for (int i = 0; i <= 5; i++) {
      fn_Connect_MQTT = connect_mqtt_server_by_gsm();
      if (fn_Connect_MQTT) {
        break;
      }
    }
  } else {
    return 0;
  }
  if (fn_Connect_MQTT) {
    for (int i = 0; i <= 3; i++) {
      fn_Subcribe_MQTT = subscribe_mqtt_via_gsm();
      if (fn_Subcribe_MQTT) {
        break;
      }
    }
    if (fn_Subcribe_MQTT)
      return 1;
    else
      stop_mqtt_via_gsm();
  }
  if (!fn_Connect_MQTT) {
    stop_mqtt_via_gsm();
  }
  return 0;
}
int stop_mqtt_via_gsm(void) {
  sendingToSimcomA76xx("AT+CMQTTDISC?\r\n");
  HAL_Delay(500);
  if (strstr((char *)rx_data_sim, "+CMQTTDISC: 0,0") != NULL) {
    printf("----------------- Connection! ------------------\n");
    AT_Check_Dis_MQTT = true;
  } else {
    printf("----------------- Disconnect! ------------------\n");
    AT_Check_Dis_MQTT = false;
    AT_Disconnect_MQTT = true;
  }
  if (AT_Check_Dis_MQTT) {
    sendingToSimcomA76xx("AT+CMQTTDISC=0,120\r\n");
    HAL_Delay(500);
    if (strstr((char *)rx_data_sim, "+CMQTTDISC: 0,0") != NULL) {
      printf("----------------- Disconnect successfully! ------------------\n");
      AT_Disconnect_MQTT = true;
    } else
      restart_stm32();
  }
  if (AT_Disconnect_MQTT) {
    sendingToSimcomA76xx("AT+CMQTTREL=0\r\n");
    HAL_Delay(500);
    if (strstr((char *)rx_data_sim, "OK") != NULL) {
      printf("----------------- Release a MQTT client successfully! "
             "------------------\n");
      AT_Rel_MQTT = true;
    } else
      restart_stm32();
  }
  if (AT_Rel_MQTT) {
    sendingToSimcomA76xx("AT+CMQTTSTOP\r\n");
    HAL_Delay(500);
    if (strstr((char *)rx_data_sim, "OK") != NULL) {
      printf("----------------- Stop MQTT service successfully! "
             "------------------\n");
      AT_Stop_MQTT = true;
      return 1;
    } else
      restart_stm32();
  }
  return 0;
}
int update_status(void) {
  for (int i = 1; i <= 3; i++) {
    fn_Publish_MQTT = publish_mqtt_via_gsm();
    if (fn_Publish_MQTT) {
      return 1;
    }
  }
  if (!fn_Publish_MQTT) {
    int temp = 0;
    for (int i = 1; i <= 15; i++) {
      temp = check_error_mqtt_via_gsm();
      if (!temp) {
        count_errors++;
        printf("-----------------UPDATE FAIL %d!------------------\n", count_errors);
        if (count_errors >= 15) {
          restart_stm32();
        }
      } else {
        count_errors = 0;
        break;
      }
    }
  }
  return 1;
}
void restart_stm32(void) {
  printf("\r\n-----------------Restart STM32------------------\r\n");
  printf("\r\n-----------------GOOD BYE !------------------\r\n");
  stop_mqtt_via_gsm();
  read_statusload();
  NVIC_SystemReset();
}
int init_cricket(void) {
  printf("\r\n-----------------INIT CRICKET !------------------\r\n");
  for (int i = 0; i <= 3; i++) {
    if (isPBDONE == true) {
      if (!fn_CheckSim) {
        fn_CheckSim = fn_check_signal_simcom();
      } else
        restart_stm32();
      if (fn_CheckSim) {
        fn_Enable_MQTT = enable_mqtt_on_gsm_modem();
      } else
        restart_stm32();
      if (fn_Enable_MQTT) {
        fn_Acquier_MQTT = acquire_gsm_mqtt_client();
      }
      if (fn_Acquier_MQTT) {
        fn_Connect_MQTT = connect_mqtt_server_by_gsm();
      }
      if (fn_Connect_MQTT) {
        fn_Subcribe_MQTT = subscribe_mqtt_via_gsm();
        if (fn_Subcribe_MQTT) {
          HAL_GPIO_WritePin(GPIOB, LED_STATUS_Pin, GPIO_PIN_SET);
          isConnectMQTT = true;
          inital_check = true;
          return 1;
        } else {
          check_error_mqtt_via_gsm();
          // isConnectedMQTT = false;
        }
      }
      printf("-----------------Complete initial check ------------------");
    }
  }
  restart_stm32();
  return 0;
}
int event_wait_function(void) {
  previousTick = HAL_GetTick();
  while (inital_check == 0 && previousTick + timeout_pb_done > HAL_GetTick()) {
    if (strstr((char *)rx_data_sim, "PB DONE")) {
      // isPBDONE = 1;
      return 1;
    }
    //		if(strstr((char *)simcomRxBuffer,"PDN ACT 1")){
    //			isPBDONE = 1;
    //			HAL_Delay(5000);
    //		}
  }
  if (isConnectSimcomA76xx == 0) {
    NVIC_SystemReset();
  }

  return 0;
}
int check_active_payload(void) {
  for (int i = 1; i <= 4; i++) {
    static int temp = 0;
    temp = HAL_GPIO_ReadPin(GPIO_LOAD_PORT[i - 1], GPIO_LOAD_PIN[i - 1]);
    if (temp == 1) {
      onReay++;
    } else
      onReay--;
  }
  if (onReay >= NUMBER_LOADS) {
    onReay = NUMBER_LOADS;
  }
  if (onReay <= 0) {
    HAL_GPIO_WritePin(ON_OFF_PWM_GPIO_Port, ON_OFF_PWM_Pin, 0);
    onReay = 0;
  }
  return onReay;
}
