/*
 * lcd1602.c
 *
 *  Created on: Oct 11, 2020
 *      Author: 10094
 */

#include "lcd1602.h"
#include <stdbool.h>
#include "stm32f1xx_hal.h"

extern EXTI_HandleTypeDef exti1;

char string_overScale[]="FATAL ERROR!";
bool overScale=false;

void lcd_write(uint8_t dat)
{
	HAL_GPIO_WritePin(LCD_D0_GPIO_Port, LCD_D0_Pin, ((dat>>0)&0x01)?GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_D1_GPIO_Port, LCD_D1_Pin, ((dat>>1)&0x01)?GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_D2_GPIO_Port, LCD_D2_Pin, ((dat>>2)&0x01)?GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_D3_GPIO_Port, LCD_D3_Pin, ((dat>>3)&0x01)?GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_D4_GPIO_Port, LCD_D4_Pin, ((dat>>4)&0x01)?GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_D5_GPIO_Port, LCD_D5_Pin, ((dat>>5)&0x01)?GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_D6_GPIO_Port, LCD_D6_Pin, ((dat>>6)&0x01)?GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_D7_GPIO_Port, LCD_D7_Pin, ((dat>>7)&0x01)?GPIO_PIN_SET:GPIO_PIN_RESET);
}

void lcd_writeCom(uint8_t com)
{
	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_RD_GPIO_Port, LCD_RD_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_RESET);

	lcd_write(com);
	HAL_Delay(1);

	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);
}

void lcd_writeData(uint8_t data)
{
	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_RD_GPIO_Port, LCD_RD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_RESET);

	lcd_write(data);
	HAL_Delay(1);

	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);
}

void lcd_init(void)
{
	lcd_writeCom(0x38);
	lcd_writeCom(0x0c);
	lcd_writeCom(0x06);
	lcd_writeCom(0x01);
	lcd_writeCom(0X80);
}

void lcd_show(uint32_t result)
{
	lcd_writeData((result / 1000)+48);
	lcd_writeData('.');
	lcd_writeData((result % 1000 / 100)+48);
	lcd_writeData((result % 100 / 10)+48);
	lcd_writeData((result % 10)+48);
	lcd_writeData('K');
	lcd_writeData('G');

	if((result>1800) && (result<10000))
	{
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

		lcd_writeCom(0x80|0x40);

		for(int i=0;i<strlen(string_overScale);i++)
		{
			lcd_writeData(string_overScale[i]);
		}
		while(1);
	}
}
