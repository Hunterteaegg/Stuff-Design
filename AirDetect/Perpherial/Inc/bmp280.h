/*
 * bmp280.h
 *
 *  Created on: Oct 14, 2020
 *      Author: hunterteaegg
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#include "main.h"
#include <stdint.h>

//BMP280 address (SDO to GND)
#define BMP280_WRITE_ADDR	0xEC
#define BMP280_READ_ADDR	0xED

//BMP280 memory map
#define	BMP280_TEMP_XLSB_REG	0xFC
#define	BMP280_TEMP_LSB_REG		0xFB
#define BMP280_TEMP_MSB_REG 	0xFA
#define	BMP280_PRESS_XLSB_REG	0xF9
#define BMP280_PRESS_LSB_REG	0xF8
#define BMP280_PRESS_MSB_REG	0xF7
#define BMP280_CONFIG_REG		0xF5
#define BMP280_CTRL_MEAS_REG	0xF4
#define	BMP280_STATUS_REG		0xF3
#define BMP280_RESET_REG		0xE0
#define BMP280_ID_REG			0xD0

//BMP280 temperature compensation reg
#define BMP280_DIG_T1_LSB_REG		0x88
#define BMP280_DIG_T1_MSB_REG		0x89
#define BMP280_DIG_T2_LSB_REG		0x8A
#define BMP280_DIG_T2_MSB_REG		0x8B
#define BMP280_DIG_T3_LSB_REG		0x8C
#define BMP280_DIG_T3_MSB_REG		0x8D

//BMP280 pressure compensation reg
#define BMP280_DIG_P1_LSB_REG		0x8E
#define BMP280_DIG_P1_MSB_REG		0x8F
#define BMP280_DIG_P2_LSB_REG		0x90
#define BMP280_DIG_P2_MSB_REG		0x91
#define BMP280_DIG_P3_LSB_REG		0x92
#define BMP280_DIG_P3_MSB_REG		0x93
#define BMP280_DIG_P4_LSB_REG		0x94
#define BMP280_DIG_P4_MSB_REG		0x95
#define BMP280_DIG_P5_LSB_REG		0x96
#define BMP280_DIG_P5_MSB_REG		0x97
#define BMP280_DIG_P6_LSB_REG		0x98
#define BMP280_DIG_P6_MSB_REG		0x99
#define BMP280_DIG_P7_LSB_REG		0x9A
#define BMP280_DIG_P7_MSB_REG		0x9B
#define BMP280_DIG_P8_LSB_REG		0x9C
#define BMP280_DIG_P8_MSB_REG		0x9D
#define BMP280_DIG_P9_LSB_REG		0x9E
#define BMP280_DIG_P9_MSB_REG		0x9F

unsigned short dig_T1;
signed short dig_T2;
signed short dig_T3;
unsigned short dig_P1;
signed short dig_P2;
signed short dig_P3;
signed short dig_P4;
signed short dig_P5;
signed short dig_P6;
signed short dig_P7;
signed short dig_P8;
signed short dig_P9;

//BMP280 power mode select (mode settings)
typedef enum{
	POWER_MODE_SLEEP=0,
	POWER_MODE_FORCE=1,
	POWER_MODE_NORMAL=3,
} BMP280_POWER_MODE ;

//BMP280 oversampling select (osrs_t and orsr_p settings)
typedef enum{
	OVERSAMPLING_SKIP=0,
	OVERSAMPLING_x1=1,
	OVERSAMPLING_x2=2,
	OVERSAMPLING_x4=3,
	OVERSAMPLING_x8=4,
	OVERSAMPLING_x16=5,
} BMP280_OVERSAMPLING;

//BMP280 standby time (t_sb settings)
typedef enum{
	STANDBY_TIME_005=0,
	STANDBY_TIME_625=1,
	STANDBY_TIME_125=2,
	STANDBY_TIME_250=3,
	STANDBY_TIME_500=4,
	STANDBY_TIME_1000=5,
	STANDBY_TIME_2000=6,
	STANDBY_TIME_4000=7,
} BMP280_STANDBY_TIME;

//BMP280 filter
typedef enum{
	FILTER_OFF=0x0,
	FILTER_MODE_1,
	FILTER_MODE_2,
	FILTER_MODE_3,
	FILTER_MODE_4,
} BMP280_FILTER_COEFFICIENT;

typedef enum{
	SPI3W_ENABLE=1,
	SPI3W_DISABLE=0,
} BMP280_SPI3W;

//BMP280 work settings
typedef struct{
	BMP280_POWER_MODE powermode;
	BMP280_OVERSAMPLING oversampling_temp;
	BMP280_OVERSAMPLING oversampling_press;
	BMP280_STANDBY_TIME standy_time;
	BMP280_FILTER_COEFFICIENT filter_coefficient;
	BMP280_SPI3W spi3w;
} BMP280_SETTINGS;

//BMP280 read data
uint32_t temperature;
uint32_t pressure;

extern I2C_HandleTypeDef hi2c2;

void BMP280_init(BMP280_SETTINGS handle);
uint32_t BMP280_temp_read(void);
uint32_t BMP280_press_read(void);
void getDig_T(void);
void getDig_P(void);
int32_t bmp280_compensate_T_int32(int32_t adc_T);
uint32_t bmp280_compensate_P_int64(int32_t adc_P);
uint32_t bmp280_compensate_P_int32(int32_t adc_P);

#endif /* INC_BMP280_H_ */


