/**
 * 20170515, Beck Pang
 * 20170525, Beck Pang, updated to 45 degree only
 * @brief observer app moving from Arduino, to get an accurate gravity vector
 * in one dimensional model
 */
#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "observer_app.h"
#include "calibration.h"

xSemaphoreHandle print_mux;

static float          body_tilt_angle_raw;
static float          body_tilt_angle_prev;
static unsigned short first_run = 1;
static void accel_tilt_estimation(void);
static void average_gyro_rate(void);
static void body_tilt_filter(void);

/**
 * @assumption: both sensor are parallel to the pivot line in xz frame
 */
static void accel_tilt_estimation(void)
{
    float length_ratio = observer_imu_length1 / observer_imu_length2;
    float y_yaw        = IMU_Z_ROLL_DEG * M_DEG_TO_RAD;
    float y_mat[9];

    roty(-y_yaw, y_mat);
    float act_45_0[3];
    float act_45_1[3];
    matrix_multiply(y_mat, &a_m_s2_act[0][0], 3, 3, 1, act_45_0);
    matrix_multiply(y_mat, &a_m_s2_act[1][0], 3, 3, 1, act_45_1);
    float m_x = act_45_0[0] - length_ratio * act_45_1[0];
    float m_z = act_45_0[2] - length_ratio * act_45_1[2];

    // // 45º calculation
    // body_tilt_angle_raw = atan2((m_x - m_z), (-m_x - m_z)) + balancing_point;
    // Parallel calculation
    // body_tilt_angle_raw = atan2(-m_x, m_z) - balancing_point * M_DEG_TO_RAD;
    body_tilt_angle_raw = atan2(m_x, -m_z);
    if (!accelerometer_not_calibrated && !gyroscope_not_calibrated)
    {
        printf("%f\t%f\t%f\t%f\t%f\t%f\t%f\n",
               body_tilt_angle_raw * M_RAD_TO_DEG,
               a_m_s2_act[0][0], a_m_s2_act[0][1], a_m_s2_act[0][2],
               act_45_0[0], act_45_0[1], act_45_0[2]);
        vTaskDelay(200 / portTICK_RATE_MS);
    }
}


/**
 * @brief rotate in xz frame, so only take the normal vector in y direction
 */
static void average_gyro_rate(void)
{
    float gyro_sum = 0;

    for (unsigned short i = 0; i < imu_count; i++)
    {
        gyro_sum += g_radian_s_act[i][1];
    }
    body_tilt_angle_dot = gyro_sum / (float)imu_count - angle_rate_offset * M_DEG_TO_RAD;
}


static void body_tilt_filter(void)
{
    // if (first_run)
    // {
    body_tilt_angle = body_tilt_angle_raw;
    //     first_run       = 0;
    // }
    // else
    // {
    //     body_tilt_angle = COMPLE_K1 * body_tilt_angle_raw +
    //                       (1 - COMPLE_K1) * (body_tilt_angle_prev + T_period * body_tilt_angle_dot);
    //     body_tilt_angle_prev = body_tilt_angle;
    // }
}


void observer_task(void *arg)
{
    uint32_t task_idx    = (uint32_t)arg;
    int      count_print = 0;

    while (1)
    {
        accel_tilt_estimation();
        average_gyro_rate();
        body_tilt_filter();


        count_print++;
        if (count_print % 10)
        {
            count_print = 0;
            // xSemaphoreTake(print_mux, portMAX_DELAY);
            // printf("[%d] %f, %f, %f\n", task_idx, body_tilt_angle_raw, body_tilt_angle_dot, body_tilt_angle );
            // // printf("Observer[%d] filtered angle: %f\n", task_idx, body_tilt_angle * M_RAD_TO_DEG);
            // printf("Observer[%d]Actual imu reading:%f\t%f\t%f\t%f\t%f\t%f\n",
            //        task_idx, a_m_s2_act[0][0], a_m_s2_act[0][1], a_m_s2_act[0][2],
            //        a_m_s2_act[1][0], a_m_s2_act[1][1], a_m_s2_act[1][2]);
            // xSemaphoreGive(print_mux);
        }


        // printf("gyro actual reading: %f\t%f\t%f\t%f\t%f\t%f\n",
        //        g_radian_s_act[0][0], g_radian_s_act[0][1], g_radian_s_act[0][2],
        //        g_radian_s_act[1][0], g_radian_s_act[1][1], g_radian_s_act[1][2]);


        vTaskDelay((int)(T_period * 1000) / portTICK_RATE_MS);
    }
}


/*
 * void app_main()
 * {
 *  print_mux = xSemaphoreCreateMutex();
 *  imu_task_init();
 *
 *  printf("finished initialization for i2c test app.\n");
 *  xTaskCreate(imu_update_task, "imu_update_task_0", 1024 * 2, (void * )0, 5, NULL);
 *  xTaskCreate(observer_task, "observer_task_0", 1024 * 2, (void * )0, 6, NULL);
 * }
 */
