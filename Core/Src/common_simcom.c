/*
 * common_simcom.c
 *
 *  Created on: Aug 28, 2024
 *      Author: thuanphat7
 */
#include "config.h"
#include "string.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"
#include "cJSON.h"
#include <main.h>

float fn_check_signal_simcom(void)
{
	sendingToSimcomA76xx("ATE0\r\n");
	HAL_Delay(200);
	sendingToSimcomA76xx("AT+CSQ\r\n");
	HAL_Delay(200);
	SignalStrength = (rx_data_sim[8] - 48) * 10 + (rx_data_sim[9] - 48);
	if(SignalStrength>=31)
	{
		rssi=-51;
	}else rssi = (SignalStrength * 2 - 113);
	isConnectSimcomA76xx = 1;
	HAL_Delay(200);
	if(strstr((char *)rx_data_sim,"OK")){
		isATOK = 1;
	}
	return rssi;
}
