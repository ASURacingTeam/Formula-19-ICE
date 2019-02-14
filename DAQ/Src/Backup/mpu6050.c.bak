/*
 * mpu6050.c
 *
 *  Created on: Dec 1, 2018
 *      Author: mohned
 */

#include "mpu6050.h"


/* Gyro sensitivities in degrees/s */
#define MPU6050_GYRO_SENS_250		((float) 131)
#define MPU6050_GYRO_SENS_500		((float) 65.5)
#define MPU6050_GYRO_SENS_1000		((float) 32.8)
#define MPU6050_GYRO_SENS_2000		((float) 16.4)

/* Acce sensitivities in g/s */
#define MPU6050_ACCE_SENS_2			((float) 16384)
#define MPU6050_ACCE_SENS_4			((float) 8192)
#define MPU6050_ACCE_SENS_8			((float) 4096)
#define MPU6050_ACCE_SENS_16		((float) 2048)

void MPU6050_Init(I2C_HandleTypeDef *hi2c,
	              MPU6050_I2CAddress address,
				  MPU6050_GyroFS gyro_FS,
				  MPU6050_AccelFS accel_FS,
				  MPU6050_SamplingRate sampling_rate,
				  MPU6050_RawValues* mpu)
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

	/*select multiplying factor that used to convert raw data to actual readings*/
	switch (gyro_FS)
		{
			case FULLSCALE_250:
				mpu->gyro_correction_param = (float)1 / MPU6050_GYRO_SENS_250;
				break;
			case FULLSCALE_500:
				mpu->gyro_correction_param = (float)1 / MPU6050_GYRO_SENS_500;
				break;
			case FULLSCALE_1000:
				mpu->gyro_correction_param = (float)1 / MPU6050_GYRO_SENS_1000;
				break;
			case FULLSCALE_2000:
				mpu->gyro_correction_param = (float)1 / MPU6050_GYRO_SENS_2000;
				break;
			default:
				break;
		}

	/*select multiplying factor that used to convert raw data to actual readings*/
	switch (accel_FS)
		{
			case FULLSCALE_250:
				mpu->accel_correction_param = (float)1 / MPU6050_ACCE_SENS_2;
				break;
			case FULLSCALE_500:
				mpu->accel_correction_param = (float)1 / MPU6050_ACCE_SENS_4;
				break;
			case FULLSCALE_1000:
				mpu->accel_correction_param = (float)1 / MPU6050_ACCE_SENS_8;
				break;
			case FULLSCALE_2000:
				mpu->gyro_correction_param = (float)1 / MPU6050_ACCE_SENS_16;
				break;
			default:
				break;
		}
}

void MPU6050_ReadRawData(I2C_HandleTypeDef *hi2c,
					  MPU6050_I2CAddress address,
					  MPU6050_RawValues* mpu)
{
    uint8_t data_buffer[7];
	/*read raw data of gyroscope*/
	data_buffer[0]=MPU6050_GYRO_XOUT_H;
	HAL_I2C_Master_Transmit(hi2c,(uint16_t)address,data_buffer,1,1000);
	HAL_I2C_Master_Receive(hi2c,(uint16_t)address,&data_buffer[1],6,1000);
	mpu->raw_gyroscope_x=(int16_t)(data_buffer[1]<<8)| data_buffer[2];
	mpu->raw_gyroscope_y=(int16_t)(data_buffer[3]<<8)| data_buffer[4];
	mpu->raw_gyroscope_z=(int16_t)(data_buffer[5]<<8)| data_buffer[6];
	HAL_Delay(20);

	/*read data of accelerometer*/
	data_buffer[0]=MPU6050_ACCEL_XOUT_H;
	HAL_I2C_Master_Transmit(hi2c,(uint16_t)address,data_buffer,1,1000);
	HAL_I2C_Master_Receive(hi2c,(uint16_t)address,&data_buffer[1],6,1000);
	mpu->raw_accelerometer_x=(int16_t)(data_buffer[1]<<8)| data_buffer[2];
	mpu->raw_accelerometer_y=(int16_t)(data_buffer[3]<<8)| data_buffer[4];
	mpu->raw_accelerometer_z=(int16_t)(data_buffer[5]<<8)| data_buffer[6];
	HAL_Delay(20);

}


