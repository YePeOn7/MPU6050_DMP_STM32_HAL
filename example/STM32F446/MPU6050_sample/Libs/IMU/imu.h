/**
 * @file imu.h
 * @brief Imu Application layer header
 * @ingroup IMU
 */

/*
 * imu.h
 *
 *  Created on: Jun 9, 2024
 *      Author: yp7
 */

#ifndef IMU_IMU_H_
#define IMU_IMU_H_
#include "main.h"

typedef struct{
	I2C_HandleTypeDef* hi2c;
	GPIO_TypeDef * GPIO_SLC;
	uint32_t GPIO_PIN_SCL;
	GPIO_TypeDef * GPIO_SDA;
	uint32_t GPIO_PIN_SDA;
}IMUSetting_t;

void imu_init(IMUSetting_t _setting);
float imu_getYaw();

#endif /* IMU_IMU_H_ */
