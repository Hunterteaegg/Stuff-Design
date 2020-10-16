/*
 * dht11.h
 *
 *  Created on: Oct 16, 2020
 *      Author: 10094
 */

#ifndef DHT11_H_
#define DHT11_H_

#include <stm32f1xx_hal.h>
#include <stm32f1xx_hal_rcc.h>

void DHT11_read(void);
void DHT11_convert(void);

#endif /* DHT11_H_ */
