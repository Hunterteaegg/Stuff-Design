/*
 * bluetooth.c
 *
 *  Created on: Oct 17, 2020
 *      Author: 10094
 */

#include "bluetooth.h"

extern UART_HandleTypeDef huart1;
extern uint8_t dht11_data[5];
extern uint16_t gy30_data;

uint8_t string_humidity[]="humidity:";

uint8_t space='\n';

void bluetooth_transmit_humi(void)
{
	uint8_t temp[2]={
			0,0,
	};

	HAL_UART_Transmit(&huart1,string_humidity , sizeof(string_humidity),0xFF);
	temp[0]=(dht11_data[0] / 10)+48;
	temp[1]=(dht11_data[0] % 10)+48;

	HAL_UART_Transmit(&huart1, temp, sizeof(temp), 0xFF);
}

void bluetooth_transmit(void)
{
	bluetooth_transmit_humi();
	HAL_UART_Transmit(&huart1, &space, sizeof(space), 0xFF);
}

