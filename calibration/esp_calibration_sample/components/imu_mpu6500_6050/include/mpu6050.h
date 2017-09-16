/*
 *	20170513, Beck Pang, MPU6050 functions
 */

#ifndef _MPU6050_H_
#define _MPU6050_H_

#include "i2c_observer.h"
#include "mpu6500_reg.h"        // Fine for 20170513, similar registers
#include "imu_typedef.h"

void mpu6050_init(uint8_t devAddr);
esp_err_t mpu6050_getMotion6(uint8_t devAddr, imu_data_t *imu_data);


#endif
