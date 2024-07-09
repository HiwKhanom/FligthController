/*
 * ICM20948.h
 *
 *  Created on: Jun 1, 2024
 *      Author: panna
 */

#ifndef INC_ICM20948_H_
#define INC_ICM20948_H_

#include "main.h"
#include "ICM20948_RegisterMap.h"

typedef struct ICM20948{
	float AccelX;
	float AccelY;
	float AccelZ;

	float GyroX;
	float GyroY;
	float GyroZ;

	float MagnX;
	float MagnY;
	float MagnZ;

	float roll;
	float pitch;
	float yaw;
}ICM20948;

void ICM20948_allRead(I2C_HandleTypeDef *hi2c, ICM20948 *DataStruct);

void ICM20948_init(I2C_HandleTypeDef *hi2c);
void AK09916_init(I2C_HandleTypeDef *hi2c);

uint8_t ICM20948_WHO_AM_I(I2C_HandleTypeDef *hi2c);
uint8_t AK09916_WHO_AM_I(I2C_HandleTypeDef *hi2c);

void ICM20948_Gyro_Calib(I2C_HandleTypeDef *hi2c);
void ICM20948_Accel_Calib(I2C_HandleTypeDef *hi2c);

void ICM20948_readAccel(I2C_HandleTypeDef *hi2c, int16_t *Data);
void ICM20948_readGyro(I2C_HandleTypeDef *hi2c, int16_t *Data);
void AK09916_readMagn(I2C_HandleTypeDef *hi2c, int16_t *Data);

void select_user_bank(I2C_HandleTypeDef *hi2c, uint8_t usr_bank);

void ICM20948_Gyro_Scale(I2C_HandleTypeDef *hi2c, gyro_scale Gscale);
void ICM20948_Accel_Scale(I2C_HandleTypeDef *hi2c, accel_scale Ascale);

void QuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

#endif /* INC_ICM20948_H_ */
