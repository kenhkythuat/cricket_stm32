/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "config.h"
#include "string.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"
#include <math.h>

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
ADC_HandleTypeDef hadc1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
GPIO_TypeDef*  GPIO_LOAD_PORT[] = {PAYLOAD_1_GPIO_Port,PAYLOAD_2_GPIO_Port,PAYLOAD_3_GPIO_Port,PAYLOAD_4_GPIO_Port};
unsigned int GPIO_LOAD_PIN[] = {PAYLOAD_1_Pin,PAYLOAD_2_Pin,PAYLOAD_3_Pin,PAYLOAD_4_Pin};


char AT_RESET[]= "AT+CRESET\r\n";
char AT_CHECK_A76XX[]= "AT\r\n";
char AT_CHECK_ESIM[]= "AT+CGREG?\r\n";
char AT_START_MQTT[]= "AT+CMQTTSTART\r\n";
char AT_ACQUIRE_CLIENT[]="AT+CMQTTACCQ=0,\"%s\",0\r\n";
char AT_CONNECT_MQTT[]="AT+CMQTTCONNECT=0,\"%s:%d\",60,1,\"%s\",\"%s\"\r\n";
char AT_SET_PUBLISH_TOPIC[]= "AT+CMQTTTOPIC=0,%d\r\n";
char AT_SET_PUBLISH_PAYLOAD[]="AT+CMQTTPAYLOAD=0,%d\r\n";
char AT_PUBLISH[]="AT+CMQTTPUB=0,1,60\r\n";
char AT_SET_SUBCRIBE_0_9_TOPIC[]="AT+CMQTTSUBTOPIC=0,%d,1\r\n";
char AT_SET_SUBCRIBE_10_18_TOPIC[]="AT+CMQTTSUBTOPIC=0,%d,1\r\n";
char AT_SUBCRIBE_TOPIC[]= "%s%d\r\n";
char AT_SUBCRIBE[]="AT+CMQTTSUB=0\r\n";
char AT_COMMAND[100];
char AT_INFORM_PAYLOAD[]="{%d:%d}\r\n";

char STATUS_PAYLOAD_ARRAY_TOTAL[]="{\"1\":0,\"2\":0,\"3\":0,\"4\":0,\"5\":0,\"6\":0,\"7\":0,\"8\":0,\"9\":0,\"10\":0,\"11\":0,\"12\":0,\"13\":0,\"14\":0,\"15\":0,\"16\":0,\"17\":0,\"18\":0}";
int timeOutConnectA76XX= 40000;
int timeOutConnectMQTT= 15000;
int isATOK= 0;
int isPBDONE= 0;
int payLoadPin,payLoadStatus;

int rxDataCouter=0;
char simcomRxBuffer[100];
int payloadIndex;
uint8_t rxData;
uint16_t rxIndex;
int loadflag = 0;
char rxBuffer[100];
int previousTick;

int isConnectSimcomA76xx= 0;
int isConnectMQTT= 0;
int sendPayloadStatusToServer;

//extern int  lenghtOfStatusPayloadArray;
int lengthOfStatusPayloadArray;
char STATUS_PAYLOAD_ARRAY_0_9[LENGTH_STATUS_PAYLOAD_0_9];
char STATUS_PAYLOAD_ARRAY_10_18[LENGTH_STATUS_PAYLOAD_10_18];
int statusOfLoad;
int testlength;
int ledStatusSendTopic=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
void turnOnA76XX(void);	//reset module sim A76XX
void processPayload(void);
int connectSimcomA76xx();
int connectMQTT();
void sendingToSimcomA76xx(char *cmd);
void ledStatus(char cmd);
int sendStatusPayloadToMQTT(void);
void informPayloadToServer(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart -> Instance == USART1)
	{
		if((rxData!='\r')&&(rxData!='\n')){
			simcomRxBuffer[rxIndex++]=rxData;
			rxDataCouter++;
		}
		else{
			if(isConnectMQTT==1){
				switch(rxDataCouter){
				case strlen(FARM)+16:
					payLoadPin = (int)simcomRxBuffer[strlen(FARM)+15] -48;
					loadflag = 1;
					break;
				case strlen(FARM)+17:
					payLoadPin = ((int)simcomRxBuffer[strlen(FARM)+15] -48)*10+((int)simcomRxBuffer[strlen(FARM)+16]-48);
					loadflag = 1;
					break;
				case 1:
					if((loadflag==1)&&(payLoadPin<=NUMBER_LOADS)){
						payLoadStatus = (int)simcomRxBuffer[0] -48;
						if((payLoadStatus==0)||(payLoadStatus==1)){
						HAL_GPIO_WritePin(GPIO_LOAD_PORT[payLoadPin-1],GPIO_LOAD_PIN[payLoadPin-1],payLoadStatus);
						}
						//informPayloadToServer();
						loadflag = 0;
					}
					break;
				case 14:
					if(strstr((char *)simcomRxBuffer,"CMQTTPUB: 0,0")){
						IWDG->KR = 0xAAAA;
						ledStatusSendTopic=1;
					}
				default:
				}
			}
			rxDataCouter=0;
			rxIndex=0;
		}
	}
	HAL_UART_Receive_IT(&huart1, &rxData, 1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if(htim->Instance == htim6.Instance)
 {
	 if(isConnectMQTT){
		 sendPayloadStatusToServer=1;
	 }
 }
 HAL_TIM_Base_Start_IT(&htim6);
}
void sendingToSimcomA76xx(char *cmd)
{
  HAL_UART_Transmit(&huart1,(uint8_t *)cmd,strlen(cmd),1000);
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  //MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1,&rxData, 1);
  HAL_TIM_Base_Start_IT(&htim6);
  turnOnA76XX();
  HAL_GPIO_WritePin(ON_OFF_PWM_GPIO_Port,ON_OFF_PWM_Pin,0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(!isConnectSimcomA76xx){
		  isConnectSimcomA76xx = connectSimcomA76xx();
	  }
	  if(!isConnectMQTT){
		  isConnectMQTT = connectMQTT();
	  }
	  //isPBDONE =test1;
	  if(sendPayloadStatusToServer == 1){
		  sendStatusPayloadToMQTT();
		  sendPayloadStatusToServer= 0;
	  }

	  blinkled();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 39999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ON_OFF_PWM_Pin|A76XX_PWRKEY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PAYLOAD_4_Pin|PAYLOAD_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PAYLOAD_2_Pin|PAYLOAD_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ON_OFF_PWM_Pin A76XX_PWRKEY_Pin */
  GPIO_InitStruct.Pin = ON_OFF_PWM_Pin|A76XX_PWRKEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PAYLOAD_4_Pin PAYLOAD_3_Pin */
  GPIO_InitStruct.Pin = PAYLOAD_4_Pin|PAYLOAD_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PAYLOAD_2_Pin PAYLOAD_1_Pin */
  GPIO_InitStruct.Pin = PAYLOAD_2_Pin|PAYLOAD_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void turnOnA76XX(void){
	HAL_GPIO_WritePin(A76XX_PWRKEY_GPIO_Port, A76XX_PWRKEY_Pin, SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(A76XX_PWRKEY_GPIO_Port, A76XX_PWRKEY_Pin, RESET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(A76XX_PWRKEY_GPIO_Port, A76XX_PWRKEY_Pin, SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(A76XX_PWRKEY_GPIO_Port, A76XX_PWRKEY_Pin, RESET);
	HAL_Delay(10000);

}
int connectSimcomA76xx(){
	previousTick =  HAL_GetTick();
	while(isConnectSimcomA76xx == 0 && previousTick  + timeOutConnectA76XX >  HAL_GetTick()){
		if(strstr((char *)simcomRxBuffer,"PB DONE")){
			isPBDONE = 1;
		}
//		if(strstr((char *)simcomRxBuffer,"PDN ACT 1")){
//			isPBDONE = 1;
//			HAL_Delay(5000);
//		}
		if(isPBDONE==1){
			HAL_Delay(3000);
			memset(simcomRxBuffer,'0',100);
			HAL_Delay(200);
			sendingToSimcomA76xx(AT_CHECK_A76XX);
			HAL_Delay(200);
			if(strstr((char *)simcomRxBuffer,"OK")){
				isATOK = 1;
			}
		}
		if(isATOK==1){
			memset(simcomRxBuffer,'0',100);
			HAL_Delay(200);
			sendingToSimcomA76xx(AT_CHECK_ESIM);
			HAL_Delay(200);
			if(strstr((char *)simcomRxBuffer,"OKGREG: 0,1")){
				isConnectSimcomA76xx = 1;
			}
		}
	}
	if(isConnectSimcomA76xx==0){
		NVIC_SystemReset();
	}

	return isConnectSimcomA76xx;
}

int connectMQTT(void){
	sendingToSimcomA76xx(AT_START_MQTT);
	HAL_Delay(200);
	sprintf(AT_COMMAND,AT_ACQUIRE_CLIENT,MQTT_CLIENT_ID);
	sendingToSimcomA76xx(AT_COMMAND);
	HAL_Delay(200);
	sprintf(AT_COMMAND,AT_CONNECT_MQTT,MQTT_HOST,MQTT_PORT,MQTT_USER,MQTT_PASS);
	sendingToSimcomA76xx(AT_COMMAND);
	HAL_Delay(200);
	for(int i=1;i<NUMBER_LOADS+1;i++){
		isConnectMQTT = 0;
		previousTick =  HAL_GetTick();
		if(i>9){
			sprintf(AT_COMMAND,AT_SET_SUBCRIBE_10_18_TOPIC,strlen(FARM)+17);
			sendingToSimcomA76xx(AT_COMMAND);
			HAL_Delay(200);
		}
		else{
			sprintf(AT_COMMAND,AT_SET_SUBCRIBE_0_9_TOPIC,strlen(FARM)+16);//
			sendingToSimcomA76xx(AT_COMMAND);
		}
		HAL_Delay(200);
		sprintf(AT_COMMAND,AT_SUBCRIBE_TOPIC,MQTT_TOPIC_ACTUATOR_CONTROL,i);
		sendingToSimcomA76xx(AT_COMMAND);
		HAL_Delay(200);
		memset(simcomRxBuffer,'0',100);
		sendingToSimcomA76xx(AT_SUBCRIBE);
		HAL_Delay(200);
		while(isConnectMQTT == 0 && previousTick  + timeOutConnectMQTT >  HAL_GetTick()){

			if(strstr((char *)simcomRxBuffer,"CMQTTSUB: 0,0")){
					isConnectMQTT=1;
			}
		}
		if(isConnectMQTT==0){
			NVIC_SystemReset();;
		}
	}
	if(isConnectMQTT==1){
		MX_IWDG_Init();
	}
	return isConnectMQTT;
}
int sendStatusPayloadToMQTT(){
	//lengthOfStatusPayloadArray = (7*NUMBER_LOADS) +(NUMBER_LOADS-1)+2;
	//STATUS_PAYLOAD_ARRAY[lengthOfStatusPayloadArray];
	if(NUMBER_LOADS<10){
		memcpy(STATUS_PAYLOAD_ARRAY_0_9,STATUS_PAYLOAD_ARRAY_TOTAL,LENGTH_STATUS_PAYLOAD_0_9-1);
		STATUS_PAYLOAD_ARRAY_0_9[LENGTH_STATUS_PAYLOAD_0_9-1] = '}';
		for(int i=1;i<NUMBER_LOADS+1;i++){
				statusOfLoad = HAL_GPIO_ReadPin(GPIO_LOAD_PORT[i-1], GPIO_LOAD_PIN[i-1]);
				STATUS_PAYLOAD_ARRAY_0_9[i*6-1] = statusOfLoad+48;
		}
		sprintf(AT_COMMAND,AT_SET_PUBLISH_TOPIC,strlen(MQTT_TOPIC_ACTUATOR_STATUS)); // Set the topic for publish message
		sendingToSimcomA76xx(AT_COMMAND);
		HAL_Delay(500);
		sprintf(AT_COMMAND,"%s\r\n",MQTT_TOPIC_ACTUATOR_STATUS);
		sendingToSimcomA76xx(AT_COMMAND);
		HAL_Delay(500);

		//sprintf(AT_COMMAND,STATUS_PAYLOAD_ARRAY_0_9,payLoadPin,payLoadStatus);
		int lengthOfInformPayload = strlen(STATUS_PAYLOAD_ARRAY_0_9);

		sprintf(AT_COMMAND,AT_SET_PUBLISH_PAYLOAD,lengthOfInformPayload);
		sendingToSimcomA76xx(AT_COMMAND);
		HAL_Delay(500);
		//sprintf(AT_COMMAND,AT_INFORM_PAYLOAD,payLoadPin,payLoadStatus);
		sendingToSimcomA76xx(STATUS_PAYLOAD_ARRAY_0_9);
		HAL_Delay(500);// Set the payload
		sendingToSimcomA76xx(AT_PUBLISH);
		HAL_Delay(500);
	}
	else{
		memcpy(STATUS_PAYLOAD_ARRAY_10_18,STATUS_PAYLOAD_ARRAY_TOTAL,LENGTH_STATUS_PAYLOAD_10_18-1);
		STATUS_PAYLOAD_ARRAY_10_18[LENGTH_STATUS_PAYLOAD_10_18-1] = '}';
		for(int i=1;i<10;i++){
			statusOfLoad = HAL_GPIO_ReadPin(GPIO_LOAD_PORT[i-1], GPIO_LOAD_PIN[i-1]);
			STATUS_PAYLOAD_ARRAY_10_18[i*6-1] = statusOfLoad+48;

		}
		int j=0;
		for(int i=10;i<NUMBER_LOADS+1;i++){
			j++;
			statusOfLoad = HAL_GPIO_ReadPin(GPIO_LOAD_PORT[i-1], GPIO_LOAD_PIN[i-1]);
			STATUS_PAYLOAD_ARRAY_10_18[i*6+j-1] = statusOfLoad+48;
		}

		sprintf(AT_COMMAND,AT_SET_PUBLISH_TOPIC,strlen(MQTT_TOPIC_ACTUATOR_STATUS)); // Set the topic for publish message
		sendingToSimcomA76xx(AT_COMMAND);
		HAL_Delay(200);
		sprintf(AT_COMMAND,"%s\r\n",MQTT_TOPIC_ACTUATOR_STATUS);
		sendingToSimcomA76xx(AT_COMMAND);
		HAL_Delay(200);

		//sprintf(AT_COMMAND,STATUS_PAYLOAD_ARRAY_0_9,payLoadPin,payLoadStatus);
		int lengthOfInformPayload = strlen(STATUS_PAYLOAD_ARRAY_10_18);

		sprintf(AT_COMMAND,AT_SET_PUBLISH_PAYLOAD,lengthOfInformPayload);
		sendingToSimcomA76xx(AT_COMMAND);
		HAL_Delay(200);
		//sprintf(AT_COMMAND,AT_INFORM_PAYLOAD,payLoadPin,payLoadStatus);
		sendingToSimcomA76xx(STATUS_PAYLOAD_ARRAY_10_18);
		HAL_Delay(200);// Set the payload
		sendingToSimcomA76xx(AT_PUBLISH);
		HAL_Delay(200);

	}

//	if(ledStatusSendTopic == 1){
//		ledStatus('W');
//		HAL_Delay(500);
//		ledStatus('G');
//		ledStatusSendTopic= 0;
//	}
	//sendPayloadStatusToServer = 0;
	return 1;
}



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
