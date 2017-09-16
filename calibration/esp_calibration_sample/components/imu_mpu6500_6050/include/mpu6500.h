/*
 *	20170513, Beck Pang, MPU6050 functions
 */

#ifndef _MPU6500_H_
#define _MPU6500_H_


#include "i2c_observer.h"
#include "imu_typedef.h"
#include "mpu6500_reg.h"        // Fine for 20170513, similar registers

void mpu6500_init(uint8_t devAddr);
esp_err_t mpu6500_getMotion6(uint8_t devAddr, imu_data_t *imu_data);

#endif
