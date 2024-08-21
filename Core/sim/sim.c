/*
 * sim.c
 *
 *  Created on: Aug 21, 2024
 *      Author: admin
 */

#include"sim.h"

void blink(int second)
{
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
	HAL_Delay(second);
}
