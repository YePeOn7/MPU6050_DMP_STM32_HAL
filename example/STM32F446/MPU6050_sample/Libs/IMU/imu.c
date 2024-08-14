/**
 * @file imu.c
 * @brief Imu Application layer source
 * @ingroup IMU
 */

/*
 * imu.c
 *
 *  Created on: Jun 9, 2024
 *      Author: yp7
 */

#include "imu.h"
#include "MPU6050.h"
#include "IOI2C.h"
//#include "common.h"

static IMUSetting_t setting = {0};

/**
 * @brief Initialize the IMU (Inertial Measurement Unit).
 * 
 * This function initializes the MPU6050 IMU sensor using the provided I2C handle.
 * It performs necessary setup steps including initialization, DMP initialization,
 * gyro calibration, and enabling gyro calibration.
 * 
 * @param hi2c The I2C handle used for communication with the IMU.
 */
void imu_init(IMUSetting_t _setting)
{
	memcpy(&setting, &_setting, sizeof(IMUSetting_t));
	IIC_Init(*setting.hi2c);
	IIC_InitLockupRecover(setting.GPIO_SLC, setting.GPIO_PIN_SCL, setting.GPIO_SDA, setting.GPIO_PIN_SDA);
	IIC_LockupRecover();
	//printf("MPU6050 is initializing....\n");
	MPU6050_initialize();
	//printf("MPU6050 has been initialized....\n");
	MPU6050_DMPInit();
//	dmp_enable_gyro_cal(0);
	//printf("DMP Initialized\n");
	MPU6050_GyroCalibration(100);
}

/**
 * @brief Get the yaw angle from the IMU.
 * 
 * This function reads the yaw angle data from the MPU6050 IMU sensor using DMP.
 * It updates internal gyro offset, motion status, and corrects the yaw angle rate.
 * 
 * @return The yaw angle in degrees.
 */
float imu_getYaw()
{
	float yawData;

	yawData = MPU6050_readDMPYaw();
//	MPU6050_updateAngleCorrector();
//
//	MPU6050_getAllGyroOffset(gyroOffset);
//	motionStatus = MPU6050_getMotionStatus(); // 0 if there is a motion
//
//	uint8_t isStable = !vRef && !wRef && motionStatus;
//	MPU6050_updateYawCorrectorRate(isStable);

	return yawData;
}
