/**
 * 20170514, Beck Pang
 * Moving all the calibration process from Arudino to ESP32
 */
#ifndef _CALIBRATION_H_
#define _CALIBRATION_H_

#include <stdbool.h>
#include "esp_log.h"
#include "imu_task.h"

/**
 * gravity constants in m/s^2
 */
#define CONSTANTS_ONE_G       9.80665f

/**
 *  @brief calibration states record
 */
#define side_count_max        6

#define detect_attempt_max    100

enum detect_orientation_return // Declare the detection enum states
{
    DETECT_ORIENTATION_TAIL_DOWN,
    DETECT_ORIENTATION_NOSE_DOWN,
    DETECT_ORIENTATION_LEFT,
    DETECT_ORIENTATION_RIGHT,
    DETECT_ORIENTATION_UPSIDE_DOWN,
    DETECT_ORIENTATION_RIGHTSIDE_UP,
    DETECT_ORIENTATION_ERROR
};

enum calibration_state  // Declare the calibration enum states
{
    STATE_DETECT_ORIENTATION,
    STATE_READ_AVERAGE,
    STATE_CALCULATION,
    STATE_CALIBRATION_DONE,
    STATE_CALIBRATION_ERROR
};

extern bool gyroscope_not_calibrated;
extern bool accelerometer_not_calibrated;
float       accel_ref[imu_count][side_count_max][3];

void calibrate_accelerometer_task(void *arg);
void calibrate_gyroscope_task(void *arg);

#endif
