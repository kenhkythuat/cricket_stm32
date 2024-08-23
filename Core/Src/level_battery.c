/*
 * level_battery.c
 *
 *  Created on: Aug 23, 2024
 *      Author: admin
 */


#include"main.h"
#include"stdio.h"
extern ADC_HandleTypeDef hadc1;
int ADC_Value;
float Level_pin;
float Percentage_pin;
float Value_Level[10];
int count =0;
float trunggian;
float map(float in, int x_inmin, int x_inmax, int x_outmin, int x_outmax)
{
	return ((in - x_inmin)*(x_outmax - x_outmin) / (x_inmax - x_inmin) + x_outmin);
}

void Level_Pin (void)
{
	HAL_ADC_PollForConversion(&hadc1, 200);
	HAL_ADC_Start(&hadc1);
	ADC_Value = HAL_ADC_GetValue(&hadc1);
	Level_pin = map(ADC_Value,0,3100,2.5,3);
	Percentage_pin = roundf(map(Level_pin,2.5,3,0,100)*100)/100;
	printf("ADC LA: %d \n",ADC_Value );
	printf("Level_pin LA: %.2f \n",Level_pin );
	printf("Percentage_pin is: %.2f \n",Percentage_pin);
	Value_Level[count] = Percentage_pin;
	count ++;
	HAL_ADC_Stop(&hadc1);
	HAL_Delay(200);
}
void aray (void)
{
    for(int i = 0; i < 9; i++){
        for(int j = 1; j < 11; j ++){
            //neu tim thay phan tu be hon phan tu dang xet thi doi cho 2 phan tu do cho nhau
            if(Value_Level[i] > Value_Level[j]){
                //hoan vi 2 phan tu
                trunggian = Value_Level[i];
                Value_Level[i] = Value_Level[j];
                Value_Level[j] = trunggian;
            }
        }
    }

}


