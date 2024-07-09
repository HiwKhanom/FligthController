/*
 * BMP280.h
 *
 *  Created on: Jun 2, 2024
 *      Author: panna
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#include "main.h"
#include "BMP280_RegisterMap.h"

typedef struct BMP280{
	uint32_t Temp;
	uint32_t press;
	uint32_t Pa;
}BMP280;

void BMP280_init(I2C_HandleTypeDef *hi2c);

uint8_t BMP280_WHO_AM_I(I2C_HandleTypeDef *hi2c);

void BMP280_readPressure(I2C_HandleTypeDef *hi2c, uint32_t *Data);
void BMP280_readTemperature(I2C_HandleTypeDef *hi2c, uint32_t *Data);

void BMP280_allRead(I2C_HandleTypeDef *hi2c, BMP280 *DataStruct);
#endif /* INC_BMP280_H_ */
