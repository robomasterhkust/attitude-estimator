/**
 * 20170515 Beck Pang
 * basic imu type def
 */

#ifndef _IMU_TYPEDEF_H_
#define _IMU_TYPEDEF_H_

typedef struct
{
    int16_t a_raw[3];

    int16_t temp;

    int16_t g_raw[3];

    int16_t m_raw[3];

    float   a_m_s2[3];

    float   g_radian_s[3];
} imu_data_t;

/* src/lib/ecl/EKF/geo.h */
#define CONSTANTS_ONE_G              9.80665f /* m/s^2		*/
#define accelerometer_sensitivity    16384.0f
#define GYRO_TO_RADIAN               4.36332f // For 2^15 = 32768 reading = 250 degree = 4.36332 radian
#define GYRO_MAX                     32768.0f

#endif
