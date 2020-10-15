/*
 * calibration.h
 *
 *  Created on: Oct 11, 2020
 *      Author: 10094
 */

#ifndef INC_CALIBRATION_H_
#define INC_CALIBRATION_H_

#include "main.h"
#include <stm32f1xx_hal.h>
#include <stdbool.h>
#include "hx711.h"
#include "lcd1602.h"

extern uint32_t offset;
extern TIM_HandleTypeDef htim2;
extern bool overScale;

#endif /* INC_CALIBRATION_H_ */
