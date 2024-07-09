/*
 * ICM20948.c
 *
 *  Created on: Jun 1, 2024
 *      Author: panna
 */

#include "main.h"
#include "math.h"
#include "ICM20948.h"
#include "ICM20948_RegisterMap.h"

#define I2C_Timeout 5

float GscaleFac;
uint16_t AscaleFac;

int16_t gyroOut[3];
int16_t accelOut[3];
int16_t magOut[3];

float beta = 0.6045998;
float zeta = 0.0;

float pitch, yaw, roll;
float deltaT = 0.0;
float sum = 0.0;
uint32_t lastUpdate = 0;
uint32_t firstUpdate = 0;
uint32_t now = 0;

float ax, ay, az, gx, gy, gz, mx, my, mz;
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float eInt[3] = {0.0f, 0.0f, 0.0f};

float lin_ax, lin_ay, lin_az;
float a12, a22, a31, a32, a33;

void ICM20948_allRead(I2C_HandleTypeDef *hi2c, ICM20948 *DataStruct){
	uint8_t readBuffer;

	select_user_bank(hi2c, usr_bank0);

	HAL_I2C_Mem_Read(hi2c, ICM20948_ADO_high, INT_STATUS_1, I2C_MEMADD_SIZE_8BIT, &readBuffer, 1, I2C_Timeout);
	if(readBuffer & 0x01){
		ICM20948_readAccel(hi2c, accelOut);
		ax = (float)accelOut[0] / AscaleFac;
		ay = (float)accelOut[1] / AscaleFac;
		az = (float)accelOut[2] / AscaleFac;

		DataStruct->AccelX = ax;
		DataStruct->AccelY = ay;
		DataStruct->AccelZ = az;

		ICM20948_readGyro(hi2c, gyroOut);
		gx = (float)gyroOut[0] / GscaleFac;
		gy = (float)gyroOut[1] / GscaleFac;
		gz = (float)gyroOut[2] / GscaleFac;

		DataStruct->GyroX = gx;
		DataStruct->GyroY = gy;
		DataStruct->GyroZ = gz;

		AK09916_readMagn(hi2c, magOut);
		mx = (float)magOut[0] * 0.15;
		my = (float)magOut[1] * 0.15;
		mz = (float)magOut[2] * 0.15;

		DataStruct->MagnX = mx;
		DataStruct->MagnY = my;
		DataStruct->MagnZ = mz;

		now = HAL_GetTick();
		deltaT = ((now - lastUpdate) / 1000.0);
		lastUpdate = now;
		sum += deltaT;

		QuaternionUpdate(ax, ay, az, gx * M_PI / 180.0f, gy * M_PI / 180.0f, gz * M_PI / 180.0f,  my,  mx, mz);

		a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
		a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
		a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
		a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
		a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

		pitch = -asinf(a32);
		roll  = atan2f(a31, a33);
		yaw   = atan2f(a12, a22);
		pitch *= 180.0f / M_PI;
		yaw   *= 180.0f / M_PI;
		yaw   += 5.53f; // Declination

		if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
		roll  *= 180.0f / M_PI;
		lin_ax = ax + a31;
		lin_ay = ay + a32;
		lin_az = az - a33;

		DataStruct->yaw = yaw;
		DataStruct->pitch = pitch;
		DataStruct->roll = roll;

		sum = 0;
	}
}

void select_user_bank(I2C_HandleTypeDef *hi2c, uint8_t usr_bank){
	HAL_I2C_Mem_Write(hi2c, ICM20948_ADO_high, REG_BANK_SEL, I2C_MEMADD_SIZE_8BIT, &usr_bank, 1, I2C_Timeout);
}

uint8_t ICM20948_WHO_AM_I(I2C_HandleTypeDef *hi2c){
	uint8_t readBuffer;

	select_user_bank(hi2c, usr_bank0);
	HAL_I2C_Mem_Read(hi2c, ICM20948_ADO_high, WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &readBuffer, 1, I2C_Timeout);

	if(readBuffer == ICM20948_ID)  return 1;
	else return 0;
}

uint8_t AK09916_WHO_AM_I(I2C_HandleTypeDef *hi2c){
	uint8_t readBuffer;

	select_user_bank(hi2c, usr_bank3);
	HAL_I2C_Mem_Read(hi2c, AK09916_addr, AK09916_WIA2, I2C_MEMADD_SIZE_8BIT, &readBuffer, 1, I2C_Timeout);

	if(readBuffer == AK09916_ID) return 1;
	else return 0;
}

void ICM20948_init(I2C_HandleTypeDef *hi2c){
	uint8_t readBuffer;
	uint8_t writeBuffer;

	while(!ICM20948_WHO_AM_I(hi2c));

	//soft reset
	writeBuffer = 0x80 | 0x41;
	HAL_I2C_Mem_Write(hi2c, ICM20948_ADO_high, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
	HAL_Delay(100);

	//wakeup
	HAL_I2C_Mem_Read(hi2c, ICM20948_ADO_high, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &readBuffer, 1, I2C_Timeout);
	writeBuffer = readBuffer & 0xBF;
	HAL_I2C_Mem_Write(hi2c, ICM20948_ADO_high, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
	HAL_Delay(100);

	//select clock source
	HAL_I2C_Mem_Read(hi2c, ICM20948_ADO_high, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &readBuffer, 1, I2C_Timeout);
	writeBuffer  = readBuffer | 0x01;
	HAL_I2C_Mem_Write(hi2c, ICM20948_ADO_high, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);

	//enable bypass mode to connect magneto to main communication line
	HAL_I2C_Mem_Read(hi2c, ICM20948_ADO_high, INT_PIN_CFG, I2C_MEMADD_SIZE_8BIT, &readBuffer, 1, I2C_Timeout);
	writeBuffer = readBuffer | 0x02;
	HAL_I2C_Mem_Write(hi2c, ICM20948_ADO_high, INT_PIN_CFG, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);

	//on prevent read data while new raw data write
	select_user_bank(hi2c, usr_bank2);
	writeBuffer  = 0x01;
	HAL_I2C_Mem_Write(hi2c, ICM20948_ADO_high, ODR_ALIGN_EN, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);

	//internal lowpass filter
	HAL_I2C_Mem_Read(hi2c, ICM20948_ADO_high, GYRO_CONFIG_1, I2C_MEMADD_SIZE_8BIT, &readBuffer, 1, I2C_Timeout);
	writeBuffer  = readBuffer | (0x00 << 3);
	HAL_I2C_Mem_Write(hi2c, ICM20948_ADO_high, GYRO_CONFIG_1, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);

	HAL_I2C_Mem_Read(hi2c, ICM20948_ADO_high, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &readBuffer, 1, I2C_Timeout);
	writeBuffer  = readBuffer | (0x00 << 3);
	HAL_I2C_Mem_Write(hi2c, ICM20948_ADO_high, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);

	//sample rate divider
	writeBuffer  = 0x00;
	HAL_I2C_Mem_Write(hi2c, ICM20948_ADO_high, GYRO_SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);

	writeBuffer  = 0x00;
	HAL_I2C_Mem_Write(hi2c, ICM20948_ADO_high, ACCEL_SMPLRT_DIV_1, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
	HAL_I2C_Mem_Write(hi2c, ICM20948_ADO_high, ACCEL_SMPLRT_DIV_2, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);

	//Gyro Accel Calibration
	ICM20948_Gyro_Calib(hi2c);
	ICM20948_Accel_Calib(hi2c);

	//set scale
	ICM20948_Accel_Scale(hi2c, _16g);
	ICM20948_Gyro_Scale(hi2c, _2000dps);
}

void AK09916_init(I2C_HandleTypeDef *hi2c){
	uint8_t writeBuffer;

	if(AK09916_WHO_AM_I(hi2c)){
		//soft reset
		writeBuffer = 0x01;
		HAL_I2C_Mem_Write(hi2c, AK09916_addr, CNTL3, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
		HAL_Delay(100);

		//select mode
		writeBuffer = continuous_measurement_100hz;
		HAL_I2C_Mem_Write(hi2c, AK09916_addr, CNTL2, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
		HAL_Delay(100);
	}
}

void ICM20948_Gyro_Calib(I2C_HandleTypeDef *hi2c){

	int16_t rawData[3] = {0};
	int32_t gyro_bias[3] = {0};
	uint8_t gyro_offset[6] = {0};

	for(uint8_t i = 0; i < 100; i++)
	{
		ICM20948_readGyro(hi2c, rawData);
		gyro_bias[0] += rawData[0];
		gyro_bias[1] += rawData[1];
		gyro_bias[2] += rawData[2];
	}

	gyro_bias[0] /= 100;
	gyro_bias[1] /= 100;
	gyro_bias[2] /= 100;

	gyro_offset[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF;
	gyro_offset[1] = (-gyro_bias[0] / 4)       & 0xFF;
	gyro_offset[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
	gyro_offset[3] = (-gyro_bias[1] / 4)       & 0xFF;
	gyro_offset[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
	gyro_offset[5] = (-gyro_bias[2] / 4)       & 0xFF;

	select_user_bank(hi2c, usr_bank2);
	HAL_I2C_Mem_Write(hi2c, ICM20948_ADO_high, XG_OFFS_USRH, I2C_MEMADD_SIZE_8BIT, &gyro_offset[0], 6, I2C_Timeout);
}

void ICM20948_Accel_Calib(I2C_HandleTypeDef *hi2c){
	int16_t rawData[3] = {0};
	uint8_t TemprawData1[2] = {0};
	uint8_t TemprawData2[2] = {0};
	uint8_t TemprawData3[2] = {0};
	int32_t accel_bias[3] = {0};
	int32_t accel_bias_reg[3] = {0};
	uint8_t accel_offset[6] = {0};

	for(uint8_t i = 0; i < 100; i++)
		{
			ICM20948_readAccel(hi2c, rawData);
			accel_bias[0] += rawData[0];
			accel_bias[1] += rawData[1];
			accel_bias[2] += rawData[2];
		}

	accel_bias[0] /= 100;
	accel_bias[1] /= 100;
	accel_bias[2] /= 100;

	uint8_t mask_bit[3] = {0, 0, 0};
	select_user_bank(hi2c, usr_bank1);

	HAL_I2C_Mem_Read(hi2c, ICM20948_ADO_high, XA_OFFS_H, I2C_MEMADD_SIZE_8BIT, &TemprawData1[0], 2, I2C_Timeout);
	accel_bias_reg[0] = (int32_t)(TemprawData1[0] << 8 | TemprawData1[1]);
	mask_bit[0] = TemprawData1[1] & 0x01;

	HAL_I2C_Mem_Read(hi2c, ICM20948_ADO_high, YA_OFFS_H, I2C_MEMADD_SIZE_8BIT, &TemprawData2[0], 2, I2C_Timeout);
	accel_bias_reg[1] = (int32_t)(TemprawData2[0] << 8 | TemprawData2[1]);
	mask_bit[1] = TemprawData2[1] & 0x01;

	HAL_I2C_Mem_Read(hi2c, ICM20948_ADO_high, ZA_OFFS_H, I2C_MEMADD_SIZE_8BIT, &TemprawData3[0], 2, I2C_Timeout);
	accel_bias_reg[2] = (int32_t)(TemprawData3[0] << 8 | TemprawData3[1]);
	mask_bit[2] = TemprawData3[1] & 0x01;

	accel_bias_reg[0] -= (accel_bias[0] / 8);
	accel_bias_reg[1] -= (accel_bias[1] / 8);
	accel_bias_reg[2] -= (accel_bias[2] / 8);

	accel_offset[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  	accel_offset[1] = (accel_bias_reg[0])      & 0xFE;
	accel_offset[1] = accel_offset[1] | mask_bit[0];

	accel_offset[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  	accel_offset[3] = (accel_bias_reg[1])      & 0xFE;
	accel_offset[3] = accel_offset[3] | mask_bit[1];

	accel_offset[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	accel_offset[5] = (accel_bias_reg[2])      & 0xFE;
	accel_offset[5] = accel_offset[5] | mask_bit[2];

	HAL_I2C_Mem_Write(hi2c, ICM20948_ADO_high, XA_OFFS_H, I2C_MEMADD_SIZE_8BIT, &accel_offset[0], 2, I2C_Timeout);
	HAL_I2C_Mem_Write(hi2c, ICM20948_ADO_high, YA_OFFS_H, I2C_MEMADD_SIZE_8BIT, &accel_offset[2], 2, I2C_Timeout);
	HAL_I2C_Mem_Write(hi2c, ICM20948_ADO_high, ZA_OFFS_H, I2C_MEMADD_SIZE_8BIT, &accel_offset[4], 2, I2C_Timeout);
}

void ICM20948_readAccel(I2C_HandleTypeDef *hi2c, int16_t *Data){
	uint8_t readAccelBuffer[6];

	select_user_bank(hi2c, usr_bank0);
	HAL_I2C_Mem_Read(hi2c, ICM20948_ADO_high, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, &readAccelBuffer[0], 6, I2C_Timeout);

	Data[0] = ((int16_t)readAccelBuffer[0] << 8 | readAccelBuffer[1]);
	Data[1] = ((int16_t)readAccelBuffer[2] << 8 | readAccelBuffer[3]);
	Data[2] = ((int16_t)readAccelBuffer[4] << 8 | readAccelBuffer[5]);
}

void ICM20948_readGyro(I2C_HandleTypeDef *hi2c, int16_t *Data){
	uint8_t readGyroBuffer[6];

	select_user_bank(hi2c, usr_bank0);
	HAL_I2C_Mem_Read(hi2c, ICM20948_ADO_high, GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, &readGyroBuffer[0], 6, I2C_Timeout);

	Data[0] = ((int16_t)readGyroBuffer[0] << 8 | readGyroBuffer[1]);
	Data[1] = ((int16_t)readGyroBuffer[2] << 8 | readGyroBuffer[3]);
	Data[2] = ((int16_t)readGyroBuffer[4] << 8 | readGyroBuffer[5]);
}

void AK09916_readMagn(I2C_HandleTypeDef *hi2c, int16_t *Data){
	uint8_t readMagnBuffer[6];
	uint8_t drdy, hofl;

	//check DRDY ready status
	HAL_I2C_Mem_Read(hi2c, AK09916_addr, ST1, I2C_MEMADD_SIZE_8BIT, &drdy, 1, I2C_Timeout);
	if((drdy & 0x01) == 0) return;

	//Read Magn data
	HAL_I2C_Mem_Read(hi2c, AK09916_addr, HXL, I2C_MEMADD_SIZE_8BIT, &readMagnBuffer[0], 6, I2C_Timeout);

	//check overflow data
	HAL_I2C_Mem_Read(hi2c, AK09916_addr, ST2, I2C_MEMADD_SIZE_8BIT, &hofl, 1, I2C_Timeout);
	if((hofl & 0x08) == 0x08) return;

	Data[0] = ((int16_t)readMagnBuffer[1] << 8) | readMagnBuffer[0];
	Data[1] = ((int16_t)readMagnBuffer[3] << 8) | readMagnBuffer[2];
	Data[2] = ((int16_t)readMagnBuffer[5] << 8) | readMagnBuffer[4];
}

void ICM20948_Gyro_Scale(I2C_HandleTypeDef *hi2c, gyro_scale Gscale){
	uint8_t renewBuffer;

	select_user_bank(hi2c, usr_bank2);
	HAL_I2C_Mem_Read(hi2c, ICM20948_ADO_high, GYRO_CONFIG_1, I2C_MEMADD_SIZE_8BIT, &renewBuffer, 1, I2C_Timeout);

	switch (Gscale) {
		case _250dps:
			renewBuffer |= 0x00;
			GscaleFac = 131.0;
			break;
		case _500dps:
			renewBuffer |= 0x02;
			GscaleFac = 65.5;
			break;
		case _1000dps:
			renewBuffer |= 0x04;
			GscaleFac = 32.8;
			break;
		case _2000dps:
			renewBuffer |= 0x06;
			GscaleFac = 16.4;
			break;
	}

	HAL_I2C_Mem_Write(hi2c, ICM20948_ADO_high, GYRO_CONFIG_1, I2C_MEMADD_SIZE_8BIT, &renewBuffer, 1, I2C_Timeout);
}

void ICM20948_Accel_Scale(I2C_HandleTypeDef *hi2c, accel_scale Ascale){
	uint8_t renewBuffer;

	select_user_bank(hi2c, usr_bank2);
	HAL_I2C_Mem_Read(hi2c, ICM20948_ADO_high, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &renewBuffer, 1, I2C_Timeout);

	switch (Ascale) {
		case _2g:
			renewBuffer |= 0x00;
			AscaleFac = 16384;
			break;
		case _4g:
			renewBuffer |= 0x02;
			AscaleFac = 8192;
			break;
		case _8g:
			renewBuffer |= 0x04;
			AscaleFac = 4096;
			break;
		case _16g:
			renewBuffer |= 0x06;
			AscaleFac = 2048;
			break;
	}
}

void QuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz){
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltaT;
    q2 += qDot2 * deltaT;
    q3 += qDot3 * deltaT;
    q4 += qDot4 * deltaT;
    norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}
