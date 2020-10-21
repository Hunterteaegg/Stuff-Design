/*
 * bluetooth.c
 *
 *  Created on: Oct 17, 2020
 *      Author: 10094
 */

#include "bluetooth.h"
#include "bmp280.h"

extern UART_HandleTypeDef huart1;
extern uint8_t dht11_data[5];
extern uint16_t gy30_data;

/*annotations*/
uint8_t string_humidity[]="humidity:";
uint8_t string_temperature[]="temperature:";
uint8_t string_luminance[]="luminance:";
uint8_t string_pressure[]="pressure:";

/*control-type characters*/
uint8_t line_feed='\n';
uint8_t space=' ';


void bluetooth_transmit_humi(void)
{
	uint8_t temp[3]={
			0,0,'%',
	};

	temp[0]=(dht11_data[0] / 10)+48;
	temp[1]=(dht11_data[0] % 10)+48;

	HAL_UART_Transmit(&huart1,string_humidity , sizeof(string_humidity),0xFF);
	HAL_UART_Transmit(&huart1, temp, sizeof(temp), 0xFF);
}

void bluetooth_transmit_temp(void)
{
	uint8_t temp[7]={
			0,0,'.',0,0,'`','C',
	};

	/*
	temp[0]=(dht11_data[2] / 10)+48;
	temp[1]=(dht11_data[2] % 10)+48;
	temp[3]=(dht11_data[3] / 100)+48;
	temp[4]=(dht11_data[3] % 100 / 10)+48;
	temp[5]=(dht11_data[3] % 10)+48;
	*/

	int32_t temp_data=bmp280_compensate_T_int32(BMP280_temp_read());

	temp[0]=(temp_data / 1000)+48;
	temp[1]=(temp_data % 1000 / 100)+48;
	temp[3]=(temp_data % 100 / 10)+48;
	temp[4]=(temp_data % 10)+48;

	HAL_UART_Transmit(&huart1, string_temperature, sizeof(string_temperature), 0xFF);
	HAL_UART_Transmit(&huart1, temp, sizeof(temp), 0xFF);
}

void bluetooth_transmit_pressure(void)
{
	uint8_t press[10]={
			0,0,0,0,'.',0,0,'h','P','a',
	};

	uint32_t press_data=bmp280_compensate_P_int32(BMP280_press_read());

	press[0]=(press_data / 100000)+48;
	press[1]=(press_data % 100000 / 10000)+48;
	press[2]=(press_data % 10000 /1000)+48;
	press[3]=(press_data % 1000 /100)+48;
	press[5]=(press_data % 100 / 10)+48;
	press[6]=(press_data % 10)+48;

	HAL_UART_Transmit(&huart1, string_pressure, sizeof(string_pressure), 0xFF);
	HAL_UART_Transmit(&huart1, press, sizeof(press), 0xFF);
}

void bluetooth_transmit_luminance(void)
{
	uint8_t temp[7]={
			0,0,0,0,0,'L','x',
	};

	temp[0]=(gy30_data / 10000)+48;
	temp[1]=(gy30_data % 10000 / 1000)+48;
	temp[2]=(gy30_data % 1000 / 100)+48;
	temp[3]=(gy30_data % 100 / 10)+48;
	temp[4]=(gy30_data % 10)+48;

	HAL_UART_Transmit(&huart1, string_luminance, sizeof(string_luminance), 0xFF);
	HAL_UART_Transmit(&huart1, temp, sizeof(temp), 0xFF);
}

void bluetooth_transmit(void)
{
	bluetooth_transmit_temp();
	HAL_UART_Transmit(&huart1, &space, sizeof(space), 0xFF);

	bluetooth_transmit_humi();
	HAL_UART_Transmit(&huart1, &space, sizeof(space), 0xFF);

	bluetooth_transmit_pressure();
	HAL_UART_Transmit(&huart1, &space, sizeof(space), 0xFF);

	bluetooth_transmit_luminance();
	HAL_UART_Transmit(&huart1, &space, sizeof(space), 0xFF);

	HAL_UART_Transmit(&huart1, &line_feed, sizeof(line_feed), 0xFF);
}

