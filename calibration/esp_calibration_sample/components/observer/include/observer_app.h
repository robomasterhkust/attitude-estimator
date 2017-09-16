/**
 * 20170515 Beck Pang
 * @brief observer typedef
 */
#ifndef _OBSERVER_APP_H_
#define _OBSERVER_APP_H_

#include "imu_task.h"
#include "matrix_math.h"
#include "control_time.h"

// K1 weight for the complementary filter
#define COMPLE_K1               0.01f

#define balancing_point         45.0f   // NOTE: Hand tuned balancing point
#define angle_rate_offset       0.06f   // NOTE: Hand tuned angle_rate_offset

#define observer_imu_length1    85.80f  // in mm
#define observer_imu_length2    196.04f // in mm

extern float a_m_s2_act[imu_count][3];

extern float g_radian_s_act[imu_count][3];

float body_tilt_angle;
float body_tilt_angle_dot;

void observer_task(void *arg);

#endif
