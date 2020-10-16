/*
 * dht11.c
 *
 *  Created on: Oct 16, 2020
 *      Author: 10094
 */

#include "dht11.h"
#include "main.h"

#define CPU_FREQUENCY_MHZ 72

uint8_t data[5]={
		0,0,0,0,0,
};
uint8_t count=1;

void delay_us(uint32_t time)
{
	uint32_t delay = (HAL_RCC_GetHCLKFreq() / 4000000 * time);
	while(delay--);
}

uint8_t DHT11_readByte(void)
{
	uint8_t data_read=0;
	uint8_t count=0;

	for(uint8_t i=0;i<8;i++)
	{
		count=1;
		while(!HAL_GPIO_ReadPin(DHT11_DOUT_GPIO_Port, DHT11_DOUT_Pin) && count++);

		delay_us(28);

		data_read<<=1;

		if(HAL_GPIO_ReadPin(DHT11_DOUT_GPIO_Port, DHT11_DOUT_Pin))
		{
			data_read |= 1;
		}

		count=1;
		while(HAL_GPIO_ReadPin(DHT11_DOUT_GPIO_Port, DHT11_DOUT_Pin) && count++);
	}

	return data_read;
}

void DHT11_read(void)
{
	HAL_GPIO_WritePin(DHT11_DOUT_GPIO_Port, DHT11_DOUT_Pin, GPIO_PIN_SET);
	delay_us(20);
	HAL_GPIO_WritePin(DHT11_DOUT_GPIO_Port, DHT11_DOUT_Pin, GPIO_PIN_RESET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(DHT11_DOUT_GPIO_Port, DHT11_DOUT_Pin, GPIO_PIN_SET);
	delay_us(20);

	if(!HAL_GPIO_ReadPin(DHT11_DOUT_GPIO_Port, DHT11_DOUT_Pin))
	{
		count=1;
		while(!HAL_GPIO_ReadPin(DHT11_DOUT_GPIO_Port, DHT11_DOUT_Pin) && count++);

		count=1;
		while(HAL_GPIO_ReadPin(DHT11_DOUT_GPIO_Port, DHT11_DOUT_Pin));

		for(uint8_t i=0;i<5;i++)
		{
			data[i]=DHT11_readByte();
		}
	}

}
