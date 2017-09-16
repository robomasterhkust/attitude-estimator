/**
 * 20170514 Beck Pang
 * moving the reusable part of calibration process to imu.h
 */
#ifndef _IMU_OBSERVER_H
#define _IMU_OBSERVER_H

#include "mpu6050.h"
#include "mpu6500.h"
#include "matrix_math.h"
#include "control_time.h"

/**
 * @source src/lib/ecl/EKF/geo.h
 */
#define M_RAD_TO_DEG       57.29578f
#define M_DEG_TO_RAD       0.01745329251994f

#define imu_count          2

// change imu frame to body frame, xz plane
#define IMU_Z_ROLL_DEG     49.5f
#define IMU_X_PITCH_DEG    -90.0f

imu_data_t imu_data[imu_count];

/**
 *	@brief accelerometers calibration process and result
 * a_m_s2_raw, raw reading from imu, change to meter per square seconds;
 * a_m_s2_sync, raw reading changed to body frame;
 * a_m_s2_act, actual reading after offset correction and affine transfer
 */
float        a_m_s2_raw[imu_count][3];
float        a_m_s2_sync[imu_count][3];
float        a_m_s2_act[imu_count][3];
extern float accel_offs[imu_count][3];
extern float accel_T[imu_count][3][3];

/**
 *	@brief gyroscope calibration process and result
 */
float        g_radian_s_raw[imu_count][3];
float        g_radian_s_sync[imu_count][3];
float        g_radian_s_act[imu_count][3];
extern float gyro_offs[imu_count][3];

void imu_update_task(void *arg);
void imu_task_init(void);


#endif
