/*
 * lcd1602.h
 *
 *  Created on: Oct 13, 2020
 *      Author: hunterteaegg
 */

#ifndef INC_LCD1602_H_
#define INC_LCD1602_H_

#include "main.h"
#include <stm32f1xx_hal.h>

void lcd_init(void);
void lcd_write(uint8_t dat);
void lcd_writeCom(uint8_t com);
void lcd_writeData(uint8_t data);
void lcd_show_dht11(uint8_t *result);
void lcd_show_gy30(uint16_t result);
void lcd_show_bmp280_temp(uint32_t result);
void lcd_show_bmp280_press(uint32_t result);

#endif /* INC_LCD1602_H_ */
