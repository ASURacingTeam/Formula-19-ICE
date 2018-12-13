/*
 * mpu6050.c
 *
 *  Created on: Dec 1, 2018
 *      Author: mohned
 */

#include "mpu6050.h"

void MPU6050_Init(I2C_HandleTypeDef *hi2c,
	              MPU6050_I2CAddress address,
				  MPU6050_GyroFS gyro_FS,
				  MPU6050_AccelFS accel_FS,
				  MPU6050_SamplingRate sampling_rate)
{

	uint8_t data_buffer[2];
	/*wake up MPU from sleep mode*/
	data_buffer[0]=MPU6050_PWR_MGMT_1;
	data_buffer[1]=SLEEPMODE_OFF;
	HAL_I2C_Master_Transmit(hi2c,(uint16_t)address,data_buffer,2,1000);

	/*set full scale of gyroscope*/
	data_buffer[0]= MPU6050_GYRO_CONFIG;
	data_buffer[1]=gyro_FS;
	HAL_I2C_Master_Transmit(hi2c,(uint16_t)address,data_buffer,2,1000);

	/*set full scale of accelerometer*/
	data_buffer[0]= MPU6050_ACCEL_CONFIG;
	data_buffer[1]=accel_FS;
	HAL_I2C_Master_Transmit(hi2c,(uint16_t)address,data_buffer,2,1000);

	/*select sampling rate*/
	data_buffer[0]=MPU6050_SMPLRT_DIV;
	data_buffer[1]=sampling_rate;
	HAL_I2C_Master_Transmit(hi2c,(uint16_t)address,data_buffer,2,1000);
}

void MPU6050_ReadRawData(I2C_HandleTypeDef *hi2c,
					  MPU6050_I2CAddress address,
					  MPU6050_RawValues* mpu)
{
	uint8_t data_buffer[7];
	/*read data of gyroscope*/
	data_buffer[0]=MPU6050_GYRO_XOUT_H;
	HAL_I2C_Master_Transmit(hi2c,(uint16_t)address,data_buffer,1,1000);
	HAL_I2C_Master_Receive(hi2c,(uint16_t)address,&data_buffer[1],6,1000);
	mpu->gyroscope_x=(int16_t)(data_buffer[1]<<8)| data_buffer[2];
	mpu->gyroscope_y=(int16_t)(data_buffer[3]<<8)| data_buffer[4];
	mpu->gyroscope_z=(int16_t)(data_buffer[5]<<8)| data_buffer[6];
	HAL_Delay(20);

	/*read data of accelerometer*/
	data_buffer[0]=MPU6050_ACCEL_XOUT_H;
	HAL_I2C_Master_Transmit(hi2c,(uint16_t)address,data_buffer,1,1000);
	HAL_I2C_Master_Receive(hi2c,(uint16_t)address,&data_buffer[1],6,1000);
	mpu->accelerometer_x=(int16_t)(data_buffer[1]<<8)| data_buffer[2];
	mpu->accelerometer_y=(int16_t)(data_buffer[3]<<8)| data_buffer[4];
	mpu->accelerometer_z=(int16_t)(data_buffer[5]<<8)| data_buffer[6];
	HAL_Delay(20);
}


