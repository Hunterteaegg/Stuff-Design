/*
 * hx711.c
 *
 *  Created on: Oct 11, 2020
 *      Author: 10094
 */


#include "hx711.h"

void delay(uint8_t i)
{
	while(i--);
}

uint32_t offset  = 21711;

uint32_t readData()
{
	uint32_t count=0;

	HAL_GPIO_WritePin(HX711_ADDO_GPIO_Port, HX711_ADDO_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HX711_ADSK_GPIO_Port, HX711_ADSK_Pin, GPIO_PIN_RESET);
	delay(1);
	while(HAL_GPIO_ReadPin(HX711_ADDO_GPIO_Port, HX711_ADDO_Pin));

	for(uint8_t i=0;i<24;i++)
	{
		HAL_GPIO_WritePin(HX711_ADSK_GPIO_Port, HX711_ADSK_Pin, GPIO_PIN_SET);
		delay(1);
		count<<=1;
		HAL_GPIO_WritePin(HX711_ADSK_GPIO_Port, HX711_ADSK_Pin, GPIO_PIN_RESET);

		if(HAL_GPIO_ReadPin(HX711_ADDO_GPIO_Port, HX711_ADDO_Pin))
		{
			count++;
		}
	}

	HAL_GPIO_WritePin(HX711_ADSK_GPIO_Port, HX711_ADSK_Pin, GPIO_PIN_SET);
	delay(2);
	count^=0x800000;
	HAL_GPIO_WritePin(HX711_ADSK_GPIO_Port, HX711_ADSK_Pin, GPIO_PIN_RESET);

	return count;
}

uint32_t convert()
{
	return ((readData()/GapVal)-offset);
}
