/*
 * gy30.c
 *
 *  Created on: Oct 17, 2020
 *      Author: 10094
 */

#include "gy30.h"

extern I2C_HandleTypeDef hi2c2;
uint16_t gy30_data=0;

void GY30_read(void)
{
	uint8_t temp[2]={
			0,0,
	};
	uint8_t GY30_readCom=0x10;

	HAL_I2C_Master_Transmit(&hi2c2, GY30_WRITE_ADDR, &GY30_readCom, sizeof(GY30_readCom), 0xFF);
	HAL_Delay(200);
	HAL_I2C_Master_Receive(&hi2c2, GY30_READ_ADDR, temp, sizeof(temp), 0xFF);

	gy30_data = (uint16_t)temp[0]<<8 | (uint16_t)temp[1];
}
