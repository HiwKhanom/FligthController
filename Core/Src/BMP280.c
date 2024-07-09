/*
 * BMP280.c
 *
 *  Created on: Jun 2, 2024
 *      Author: panna
 */

#include "main.h"
#include "BMP280.h"
#include "BMP280_RegisterMap.h"

#define I2C_Timeout 5

typedef struct{
	int16_t dig_T1;
	uint16_t dig_T2;
	uint16_t dig_T3;

	int16_t dig_P1;
	uint16_t dig_P2;
	uint16_t dig_P3;
	uint16_t dig_P4;
	uint16_t dig_P5;
	uint16_t dig_P6;
	uint16_t dig_P7;
	uint16_t dig_P8;
	uint16_t dig_P9;

	float var1;
	float var2;
	float p;
	uint32_t tfine;

}CalibData;

CalibData calibData;

void BMP280_init(I2C_HandleTypeDef *hi2c){
	uint8_t writeBuffer;

	//soft reset
	writeBuffer = 0xB6;
	HAL_I2C_Mem_Write(hi2c, BMP280_addr, BMP280_reset, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
	HAL_Delay(100);

	//set oversampling press, temp power mode
	writeBuffer = 0xEB;
	HAL_I2C_Mem_Write(hi2c, BMP280_addr, BMP280_ctrl_meas, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);

	//set config
	writeBuffer = 0x0E;
	HAL_I2C_Mem_Write(hi2c, BMP280_addr, BMP280_config, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
}

uint8_t BMP280_WHO_AM_I(I2C_HandleTypeDef *hi2c){
	uint8_t readBuffer;
	HAL_I2C_Mem_Read(hi2c, BMP280_addr, BMP280_who_am_i, I2C_MEMADD_SIZE_8BIT, &readBuffer, 1, I2C_Timeout);
	if(readBuffer == BMP280_ID) return 1;
	else return 0;
}

void BMP280_readPressure(I2C_HandleTypeDef *hi2c, uint32_t *Data){
	uint8_t readPressBuffer[3];

	HAL_I2C_Mem_Read(hi2c, BMP280_addr, BMP280_press_msb, I2C_MEMADD_SIZE_8BIT, &readPressBuffer[0], 3, I2C_Timeout);
	*Data = (uint32_t)((readPressBuffer[0] << 12) | (readPressBuffer[1] << 4)  |(readPressBuffer[2] >> 4));
}

void BMP280_readTemperature(I2C_HandleTypeDef *hi2c, uint32_t *Data){
	uint8_t readTempBuffer[3];

	HAL_I2C_Mem_Read(hi2c, BMP280_addr, BMP280_temp_msb, I2C_MEMADD_SIZE_8BIT, &readTempBuffer[0], 3, I2C_Timeout);
	*Data = (uint32_t)((readTempBuffer[0] << 9) | (readTempBuffer[1] << 1) |(readTempBuffer[2] >> 7));
}

void BMP280_allRead(I2C_HandleTypeDef *hi2c, BMP280 *DataStruct){
	uint8_t readBuffer;
	uint8_t readCalibBuffer[24];

	uint32_t PrePressure;
	uint32_t PreTemp;

	HAL_I2C_Mem_Read(hi2c, BMP280_addr, BMP280_status, I2C_MEMADD_SIZE_8BIT, &readBuffer, 1, I2C_Timeout);

	if((readBuffer & 0x08) == 0x00){
		BMP280_readPressure(hi2c, &PrePressure);
		BMP280_readTemperature(hi2c, &PreTemp);

		HAL_I2C_Mem_Read(hi2c, BMP280_addr, BMP280_calib00, I2C_MEMADD_SIZE_8BIT, &readCalibBuffer[0], 24, I2C_Timeout);
		calibData.dig_T1 = (uint16_t)((readCalibBuffer[1] << 8) | readCalibBuffer[0]);
		calibData.dig_T2 = (uint16_t)((readCalibBuffer[3] << 8) | readCalibBuffer[2]);
		calibData.dig_T3 = (uint16_t)((readCalibBuffer[5] << 8) | readCalibBuffer[4]);
		calibData.dig_P1 = (uint16_t)((readCalibBuffer[7] << 8) | readCalibBuffer[6]);
		calibData.dig_P2 = (uint16_t)((readCalibBuffer[9] << 8) | readCalibBuffer[8]);
		calibData.dig_P3 = (uint16_t)((readCalibBuffer[11] << 8) | readCalibBuffer[10]);
		calibData.dig_P4 = (uint16_t)((readCalibBuffer[13] << 8) | readCalibBuffer[12]);
		calibData.dig_P5 = (uint16_t)((readCalibBuffer[15] << 8) | readCalibBuffer[14]);
		calibData.dig_P6 = (uint16_t)((readCalibBuffer[17] << 8) | readCalibBuffer[16]);
		calibData.dig_P7 = (uint16_t)((readCalibBuffer[19] << 8) | readCalibBuffer[18]);
		calibData.dig_P8 = (uint16_t)((readCalibBuffer[21] << 8) | readCalibBuffer[20]);
		calibData.dig_P9 = (uint16_t)((readCalibBuffer[23] << 8) | readCalibBuffer[22]);

		//Cal Temp
		calibData.var1 = (((double)PreTemp)/16384.0 - ((double)calibData.dig_T1)/1024.0) * ((double)calibData.dig_T2);
		calibData.var2 = ((((double)PreTemp)/131072.0 - ((double)calibData.dig_T1)/8192.0) * (((double)PreTemp)/131072.0 - ((double)calibData.dig_T1)/8192.0)) * ((double)calibData.dig_T3);
		calibData.tfine = (uint32_t)(calibData.var1 + calibData.var2);
		DataStruct->Temp = (calibData.var1 + calibData.var2) / 5120.0;

		//Cal Pressure
		calibData.var1 = ((double)calibData.tfine/2.0) - 64000.0;
		calibData.var2 = calibData.var1 * calibData.var1 * ((double)calibData.dig_P6)/32768.0;
		calibData.var2 = calibData.var2 + calibData.var1 * ((double)calibData.dig_P5) * 2.0;
		calibData.var2 = (calibData.var2/4.0) + (((double)calibData.dig_P4) * 65536.0);
		calibData.var1 = (((double)calibData.dig_P3) * calibData.var1 * calibData.var1 / 524288.0 + ((double)calibData.dig_P2)*calibData.var1)/524288.0;
		calibData.var1 = (1.0 + calibData.var1/32768.0) * ((double)calibData.dig_P1);
		calibData.p = 1048576.0 - (double)PrePressure;
		calibData.p = (calibData.p - (calibData.var2/4096.0)) * 6250.0 / calibData.var1;
		calibData.var1 = ((double)calibData.dig_P9) * calibData.p * calibData.p / 2147483648.0;
		calibData.var2 = calibData.p * ((double)calibData.dig_P8) / 32768.0;
		calibData.p = calibData.p + (calibData.var1 + calibData.var2 + ((double)calibData.dig_P7)) / 16.0;
		DataStruct->press = calibData.p;
	}
}
