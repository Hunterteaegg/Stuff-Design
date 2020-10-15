/*
 * hx711.h
 *
 *  Created on: Oct 11, 2020
 *      Author: 10094
 */

#ifndef INC_HX711_H_
#define INC_HX711_H_

#include "main.h"
#include <stm32f1xx_hal.h>
#include "stdint.h"

#define GapVal 385

uint32_t readData();
uint32_t convert();
void delay(uint8_t i);

#endif /* INC_HX711_H_ */
