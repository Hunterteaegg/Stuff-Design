/*
 * gy30.h
 *
 *  Created on: Oct 17, 2020
 *      Author: 10094
 */

#ifndef GY30_H_
#define GY30_H_

#include "main.h"
#include <stm32f1xx_hal.h>
#include <stm32f1xx_hal_i2c.h>

#define GY30_WRITE_ADDR		0x46
#define GY30_READ_ADDR		0x47

uint16_t GY30_read(void);

#endif /* GY30_H_ */
