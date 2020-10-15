/*
 * calibration.c
 *
 *  Created on: Oct 11, 2020
 *      Author: 10094
 */

#include "calibration.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
	case GPIO_PIN_1:
		if(HAL_GPIO_ReadPin(calibration_GPIO_Port, calibration_Pin)==GPIO_PIN_RESET)
		{
			offset+=convert();
		}
		break;
	default:
		;
	}
}
