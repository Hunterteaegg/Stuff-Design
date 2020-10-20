/*
 * bluetooth.c
 *
 *  Created on: Oct 20, 2020
 *      Author: 10094
 */
#include "bluetooth.h"

extern UART_HandleTypeDef huart1;

void bluetooth_transmit_weight(uint32_t weight)
{
	uint8_t display[8]={
			0,'.',0,0,0,'K','G',0x0A,
	};

	display[0]=(weight / 1000)+48;
	display[2]=(weight % 1000 / 100)+48;
	display[3]=(weight % 100 / 10)+48;
	display[4]=(weight % 10)+48;


	HAL_UART_Transmit(&huart1, display, sizeof(display), 0xFF);
}

