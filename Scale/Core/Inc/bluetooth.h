/*
 * bluetooth.h
 *
 *  Created on: Oct 20, 2020
 *      Author: 10094
 */

#ifndef INC_BLUETOOTH_H_
#define INC_BLUETOOTH_H_

#include "stm32f1xx_hal.h"
#include "main.h"
#include <stdint.h>

void bluetooth_transmit_weight(uint32_t weight);

#endif /* INC_BLUETOOTH_H_ */
