/*
 * bmp280.c
 *
 *  Created on: Oct 14, 2020
 *      Author: hunterteaegg
 */
#include "bmp280.h"

//#include "bmp280.h"
extern I2C_HandleTypeDef hi2c2;

void BMP280_init(BMP280_SETTINGS handle)
{
	uint8_t config=((uint8_t)(handle.standy_time)<<5) | ((uint8_t)(handle.filter_coefficient)<<2) | ((uint8_t)(handle.spi3w));
	uint8_t ctrl_meas=((uint8_t)(handle.oversampling_temp)<<5) | ((uint8_t)(handle.oversampling_press)<<2) | ((uint8_t)(handle.powermode));

	HAL_I2C_Mem_Write(&hi2c2, BMP280_WRITE_ADDR, BMP280_CONFIG_REG, I2C_MEMADD_SIZE_8BIT, &config, sizeof(config), 0xFF);
	HAL_I2C_Mem_Write(&hi2c2, BMP280_WRITE_ADDR, BMP280_CTRL_MEAS_REG, I2C_MEMADD_SIZE_8BIT, &ctrl_meas, sizeof(ctrl_meas), 0xFF);
}

uint32_t BMP280_temp_read(void)
{
	uint32_t temp=0;
	uint8_t temp_reg[3]={
			0,0,0
	};

	HAL_I2C_Mem_Read(&hi2c2, BMP280_READ_ADDR, BMP280_TEMP_MSB_REG, I2C_MEMADD_SIZE_8BIT, temp_reg, sizeof(temp_reg), 0xFF);

	uint32_t temp_msb=temp_reg[0];
	uint32_t temp_lsb=temp_reg[1];
	uint32_t temp_xlsb=temp_reg[2];

	temp=(temp_xlsb>>4) | (temp_lsb<<4) | (temp_msb<<12);

	return temp;
}

void getDig_T(void)
{
	uint8_t temp_lsb=0;
	uint8_t temp_msb=0;

	HAL_I2C_Mem_Read(&hi2c2, BMP280_READ_ADDR, BMP280_DIG_T1_LSB_REG, I2C_MEMADD_SIZE_8BIT, &temp_lsb, sizeof(temp_lsb), 0xFF);
	HAL_I2C_Mem_Read(&hi2c2, BMP280_READ_ADDR, BMP280_DIG_T1_MSB_REG, I2C_MEMADD_SIZE_8BIT, &temp_msb, sizeof(temp_msb), 0xFF);
	dig_T1= temp_lsb | (((uint16_t)temp_msb)<<8);

	HAL_I2C_Mem_Read(&hi2c2, BMP280_READ_ADDR, BMP280_DIG_T2_LSB_REG, I2C_MEMADD_SIZE_8BIT, &temp_lsb, sizeof(temp_lsb), 0xFF);
	HAL_I2C_Mem_Read(&hi2c2, BMP280_READ_ADDR, BMP280_DIG_T2_MSB_REG, I2C_MEMADD_SIZE_8BIT, &temp_msb, sizeof(temp_msb), 0xFF);
	dig_T2= temp_lsb | (((uint16_t)temp_msb)<<8);

	HAL_I2C_Mem_Read(&hi2c2, BMP280_READ_ADDR, BMP280_DIG_T3_LSB_REG, I2C_MEMADD_SIZE_8BIT, &temp_lsb, sizeof(temp_lsb), 0xFF);
	HAL_I2C_Mem_Read(&hi2c2, BMP280_READ_ADDR, BMP280_DIG_T3_MSB_REG, I2C_MEMADD_SIZE_8BIT, &temp_msb, sizeof(temp_msb), 0xFF);
	dig_T3= temp_lsb | (((uint16_t)temp_msb)<<8);

}

int32_t t_fine;
int32_t bmp280_compensate_T_int32(int32_t adc_T)
{
	int32_t var1,var2,T;

	var1=((((adc_T>>3)-((int32_t)dig_T1<<1)))*((int32_t)dig_T2))>>11;
	var2=(((((adc_T>>4)-((int32_t)dig_T1))*((adc_T>>4)-((int32_t)dig_T1)))>>12)*((int32_t)dig_T3))>>14;
	t_fine=var1+var2;
	T=(t_fine*5+128)>>8;

	return T;
}

uint32_t BMP280_press_read(void)
{
	uint32_t press=0;
	uint8_t press_reg[3]={
			0,0,0,
	};

	HAL_I2C_Mem_Read(&hi2c2, BMP280_READ_ADDR, 0xF7, I2C_MEMADD_SIZE_8BIT, press_reg, sizeof(press_reg), 0xFF);

	uint32_t press_msb=press_reg[0];
	uint32_t press_lsb=press_reg[1];
	uint32_t press_xlsb=press_reg[2];

	press= (press_xlsb>>4) | (press_lsb<<4) | (press_msb<<12);

	return press;
}

void getDig_P(void)
{
	uint8_t press_lsb=0;
	uint8_t press_msb=0;

	HAL_I2C_Mem_Read(&hi2c2, BMP280_READ_ADDR, BMP280_DIG_P1_LSB_REG, I2C_MEMADD_SIZE_8BIT, &press_lsb, sizeof(press_lsb), 0xFF);
	HAL_I2C_Mem_Read(&hi2c2, BMP280_READ_ADDR, BMP280_DIG_P1_MSB_REG, I2C_MEMADD_SIZE_8BIT, &press_msb, sizeof(press_msb), 0xFF);
	dig_P1= press_lsb | (((uint16_t)press_msb)<<8);

	HAL_I2C_Mem_Read(&hi2c2, BMP280_READ_ADDR, BMP280_DIG_P2_LSB_REG, I2C_MEMADD_SIZE_8BIT, &press_lsb, sizeof(press_lsb), 0xFF);
	HAL_I2C_Mem_Read(&hi2c2, BMP280_READ_ADDR, BMP280_DIG_P2_MSB_REG, I2C_MEMADD_SIZE_8BIT, &press_msb, sizeof(press_msb), 0xFF);
	dig_P2= press_lsb | (((uint16_t)press_msb)<<8);

	HAL_I2C_Mem_Read(&hi2c2, BMP280_READ_ADDR, BMP280_DIG_P3_LSB_REG, I2C_MEMADD_SIZE_8BIT, &press_lsb, sizeof(press_lsb), 0xFF);
	HAL_I2C_Mem_Read(&hi2c2, BMP280_READ_ADDR, BMP280_DIG_P3_MSB_REG, I2C_MEMADD_SIZE_8BIT, &press_msb, sizeof(press_msb), 0xFF);
	dig_P3= press_lsb | (((uint16_t)press_msb)<<8);

	HAL_I2C_Mem_Read(&hi2c2, BMP280_READ_ADDR, BMP280_DIG_P4_LSB_REG, I2C_MEMADD_SIZE_8BIT, &press_lsb, sizeof(press_lsb), 0xFF);
	HAL_I2C_Mem_Read(&hi2c2, BMP280_READ_ADDR, BMP280_DIG_P4_MSB_REG, I2C_MEMADD_SIZE_8BIT, &press_msb, sizeof(press_msb), 0xFF);
	dig_P4= press_lsb | (((uint16_t)press_msb)<<8);

	HAL_I2C_Mem_Read(&hi2c2, BMP280_READ_ADDR, BMP280_DIG_P5_LSB_REG, I2C_MEMADD_SIZE_8BIT, &press_lsb, sizeof(press_lsb), 0xFF);
	HAL_I2C_Mem_Read(&hi2c2, BMP280_READ_ADDR, BMP280_DIG_P5_MSB_REG, I2C_MEMADD_SIZE_8BIT, &press_msb, sizeof(press_msb), 0xFF);
	dig_P5= press_lsb | (((uint16_t)press_msb)<<8);

	HAL_I2C_Mem_Read(&hi2c2, BMP280_READ_ADDR, BMP280_DIG_P6_LSB_REG, I2C_MEMADD_SIZE_8BIT, &press_lsb, sizeof(press_lsb), 0xFF);
	HAL_I2C_Mem_Read(&hi2c2, BMP280_READ_ADDR, BMP280_DIG_P6_MSB_REG, I2C_MEMADD_SIZE_8BIT, &press_msb, sizeof(press_msb), 0xFF);
	dig_P6= press_lsb | (((uint16_t)press_msb)<<8);

	HAL_I2C_Mem_Read(&hi2c2, BMP280_READ_ADDR, BMP280_DIG_P7_LSB_REG, I2C_MEMADD_SIZE_8BIT, &press_lsb, sizeof(press_lsb), 0xFF);
	HAL_I2C_Mem_Read(&hi2c2, BMP280_READ_ADDR, BMP280_DIG_P7_MSB_REG, I2C_MEMADD_SIZE_8BIT, &press_msb, sizeof(press_msb), 0xFF);
	dig_P7= press_lsb | (((uint16_t)press_msb)<<8);

	HAL_I2C_Mem_Read(&hi2c2, BMP280_READ_ADDR, BMP280_DIG_P8_LSB_REG, I2C_MEMADD_SIZE_8BIT, &press_lsb, sizeof(press_lsb), 0xFF);
	HAL_I2C_Mem_Read(&hi2c2, BMP280_READ_ADDR, BMP280_DIG_P8_MSB_REG, I2C_MEMADD_SIZE_8BIT, &press_msb, sizeof(press_msb), 0xFF);
	dig_P8= press_lsb | (((uint16_t)press_msb)<<8);

	HAL_I2C_Mem_Read(&hi2c2, BMP280_READ_ADDR, BMP280_DIG_P9_LSB_REG, I2C_MEMADD_SIZE_8BIT, &press_lsb, sizeof(press_lsb), 0xFF);
	HAL_I2C_Mem_Read(&hi2c2, BMP280_READ_ADDR, BMP280_DIG_P9_MSB_REG, I2C_MEMADD_SIZE_8BIT, &press_msb, sizeof(press_msb), 0xFF);
	dig_P9= press_lsb | (((uint16_t)press_msb)<<8);
}

uint32_t bmp280_compensate_P_int64(int32_t adc_P)
{
	int64_t var1,var2,p;
	var1=((int64_t)t_fine)-128000;
	var2=var1*var1*(int64_t)dig_P6;
	var2=var2+((var1*(int64_t)dig_P5)<<17);
	var2=var2+(((int64_t)dig_P4)<<35);
	var1=((var1*var1*(int64_t)dig_P3)>>8)+((var1*(int64_t)dig_P2)<<12);
	var1=(((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
	if(var1==0)
	{
		return 0;
	}
	p=1048576-adc_P;
	p=(((p<<31)-var2)*3125)/var1;
	var1=(((int64_t)dig_P9)*(p>>13)*(p>>13))>>25;
	var2=(((int64_t)dig_P8)*p)>>19;
	p=((p+var1+var2)>>8)+(((int64_t)dig_P7)<<4);

	return (uint32_t)p;
}

uint32_t bmp280_compensate_P_int32(int32_t adc_P)
{
	int32_t var1,var2;
	uint32_t p;
	var1=(((int32_t)t_fine)>>1)-(int32_t)64000;
	var2=(((var1>>2)*(var1>>2))>>11)*((int32_t)dig_P6);
	var2=var2+((var1*((int32_t)dig_P5))<<1);
	var2=(var2>>2)+(((int32_t)dig_P4)<<16);
	var1=(((dig_P3*(((var1>>2)*(var1>>2))>>13))>>3)+((((int32_t)dig_P2)*var1)>>1))>>18;
	var1=((((32768+var1))*((int32_t)dig_P1))>>15);
	if(var1==0)
	{
		return 0;
	}
	p=(((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
	if(p<0x80000000)
	{
		p=(p<<1)/((uint32_t)var1);
	}
	else
	{
		p=(p/(uint32_t)var1)*2;
	}
	var1=(((int32_t)dig_P9)*((int32_t)(((p>>3)*(p>>3))>>13)))>>12;
	var2=(((int32_t)(p>>2))*((int32_t)dig_P8))>>13;
	p=(uint32_t)((int32_t)p+((var1+var2+dig_P7)>>4));

	return p;
}
