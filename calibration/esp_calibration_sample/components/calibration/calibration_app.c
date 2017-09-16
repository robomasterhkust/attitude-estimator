/*  First program on ESP32, Beck Pang, 20170215, 20170503
 *  Implementing a gravitational vector using six accelerometers
 *  S. Trimpe and R. D’Andrea, “Accelerometer-based tilt estimation of
 *  a rigid body with only rotational degrees of freedom,” in Robotics
 *  and Automation (ICRA), 2010 IEEE International Conference
 */

#include "calibration.h"
#include "timercmp.h"
#include "matrix_math.h"

#define DEBUG    true

static const char *TAG = "calibration_app";
static bool       side_data_collected[side_count_max] = { false, false, false, false, false, false };
static unsigned   side_complete_count          = 0;
static unsigned   detect_attempt               = 0;
static unsigned   calibration_state_record     = 0;
bool              gyroscope_not_calibrated     = true;
bool              accelerometer_not_calibrated = true;

extern imu_data_t imu_data[imu_count];

/**
 * @brief detect the accelerometer direction
 * @require a_m_s2_sync after syncronized
 */
static enum detect_orientation_return detect_orientation(uint32_t task_idx, float g)
{
    float accel_ema[3]     = { 0.0f, 0.0f, 0.0f }; // exponential moving average of accel
    float accel_disp[3]    = { 0.0f, 0.0f, 0.0f }; // max-hold disperision of accel
    float ema_len          = 0.5f;                 // exponential moving average time constant in seconds
    float normal_still_thr = 0.25;                 // normal still threshold
    float still_thr2       = powf(normal_still_thr * 3, 2);
    float accel_err_thr    = 5.0f;                 // set accelerometer error threshold to 5m/s^2


    struct timeval still_time_timeval = { .tv_sec = 0, .tv_usec = 500000, };
    struct timeval timeout_timeval    = { .tv_sec = 30, .tv_usec = 0, };
    struct timeval t_start_timeval;
    struct timeval t_timeval;

    gettimeofday(&t_start_timeval, NULL);
    gettimeofday(&t_timeval, NULL);
#if DEBUG
    printf("Current time reading in secs: %ld micros: %ld \n", t_start_timeval.tv_sec, t_start_timeval.tv_usec);
#endif

    unsigned long still_time = timerlong(&still_time_timeval);
    unsigned long t_start    = timerlong(&t_start_timeval);
    unsigned long timeout    = timerlong(&timeout_timeval);
    unsigned long t_timeout  = t_start + timeout;
    unsigned long t          = t_start;
    unsigned long t_prev     = t_start;
    unsigned long t_still    = t_start;

    bool reach_still = false;

    unsigned poll_err_count = 0;

    while (t <= (t_still + still_time) || !reach_still)
    {
        // 1. calculate the weight for exponential moving average
        gettimeofday(&t_timeval, NULL);
        t = timerlong(&t_timeval);
        float dt = (float)(t - t_prev) / 1000000.0f;
        t_prev = t;
        float weight = dt / ema_len;

        // 2. calculate the exponential moving average and max-hold disperision
        for (unsigned short i = 0; i < 3; i++)
        {
            float di = a_m_s2_sync[0][i];
            float d  = di - accel_ema[i];
            accel_ema[i] += d * weight;
            d             = d * d;
            accel_disp[i] = accel_disp[i] * (1.0f - weight);
            if (d > still_thr2 * 8.0f)
            {
                d = still_thr2 * 8.0f;
            }
            if (d > accel_disp[i])
            {
                accel_disp[i] = d;
            }
        }

        // 3. still detector with hysteresis
        if ((accel_disp[0] < still_thr2) &&
            (accel_disp[1] < still_thr2) &&
            (accel_disp[2] < still_thr2))
        {
            if (!reach_still)
            {
                ESP_LOGI(TAG, "[%d] rest position reached, hold still", task_idx);
                t_still     = t;
                t_timeout   = t + timeout;
                reach_still = true;
            }
        }
        else if ((accel_disp[0] > still_thr2 * 4.0f) ||
                 (accel_disp[1] > still_thr2 * 4.0f) ||
                 (accel_disp[2] > still_thr2 * 4.0f))
        {
            if (reach_still)
            {
                ESP_LOGI(TAG, "[%d] detected motion, hold still", task_idx);
                reach_still = false;
                vTaskDelay(200 / portTICK_RATE_MS);
            }
        }

        if (t > t_timeout)
        {
            poll_err_count++;
        }
        if (poll_err_count > 1000)
        {
            ESP_LOGI(TAG, "Detection for accelerometer failed, abort");
            return DETECT_ORIENTATION_ERROR;
        }
    }

    if ((fabsf(accel_ema[0] - CONSTANTS_ONE_G) < accel_err_thr) &&
        (fabsf(accel_ema[1]) < accel_err_thr) &&
        (fabsf(accel_ema[2]) < accel_err_thr))
    {
        return DETECT_ORIENTATION_TAIL_DOWN;            // [ g, 0, 0 ]
    }
    if ((fabsf(accel_ema[0] + CONSTANTS_ONE_G) < accel_err_thr) &&
        (fabsf(accel_ema[1]) < accel_err_thr) &&
        (fabsf(accel_ema[2]) < accel_err_thr))
    {
        return DETECT_ORIENTATION_NOSE_DOWN;            // [ -g, 0, 0 ]
    }
    if ((fabsf(accel_ema[0]) < accel_err_thr) &&
        (fabsf(accel_ema[1] - CONSTANTS_ONE_G) < accel_err_thr) &&
        (fabsf(accel_ema[2]) < accel_err_thr))
    {
        return DETECT_ORIENTATION_LEFT;            // [ 0, g, 0 ]
    }
    if ((fabsf(accel_ema[0]) < accel_err_thr) &&
        (fabsf(accel_ema[1] + CONSTANTS_ONE_G) < accel_err_thr) &&
        (fabsf(accel_ema[2]) < accel_err_thr))
    {
        return DETECT_ORIENTATION_RIGHT;            // [ 0, -g, 0 ]
    }
    if ((fabsf(accel_ema[0]) < accel_err_thr) &&
        (fabsf(accel_ema[1]) < accel_err_thr) &&
        (fabsf(accel_ema[2] - CONSTANTS_ONE_G) < accel_err_thr))
    {
        return DETECT_ORIENTATION_UPSIDE_DOWN;            // [ 0, 0, g ]
    }
    if ((fabsf(accel_ema[0]) < accel_err_thr) &&
        (fabsf(accel_ema[1]) < accel_err_thr) &&
        (fabsf(accel_ema[2] + CONSTANTS_ONE_G) < accel_err_thr))
    {
        return DETECT_ORIENTATION_RIGHTSIDE_UP;            // [ 0, 0, -g ]
    }

    return DETECT_ORIENTATION_ERROR;
}


/**
 * @brief reading the average reading from samples_num,
 *        fill the accel_ref for a single orientation
 * @require a_m_s2_sync updated on time
 * @TODO: include vibration detection in the future
 */
static bool read_accelerometer_avg(unsigned orient, uint32_t samples_num)
{
    uint32_t counts[imu_count]       = { 0 };
    double   accel_sum[imu_count][3] = { 0 };

    // unsigned err_count = 0;

    while (counts[0] < samples_num)
    {
        for (unsigned short i = 0; i < imu_count; i++)
        {
            for (unsigned short j = 0; j < 3; j++)
            {
                accel_sum[i][j] += a_m_s2_sync[i][j];
            }
            counts[i]++;
        }
    }
    for (unsigned short i = 0; i < imu_count; i++)
    {
        for (unsigned short j = 0; j < 3; j++)
        {
            accel_ref[i][orient][j] = accel_sum[i][j] / counts[i];
        }
    }
#if DEBUG
    printf("On orient %d, the accel_ref for \n the first imu: %f\t%f\t%f\n  the second imu: %f\t%f\t%f\n", orient,
           accel_ref[0][orient][0], accel_ref[0][orient][1], accel_ref[0][orient][2],
           accel_ref[1][orient][0], accel_ref[1][orient][1], accel_ref[1][orient][2]
           );
#endif

    return true;
}


static bool read_gyrscope_avg(uint32_t samples_num)
{
    uint32_t counts[imu_count]      = { 0 };
    double   gyro_sum[imu_count][3] = { 0 };

    // unsigned err_count = 0;

    while (counts[0] < samples_num)
    {
        for (unsigned short i = 0; i < imu_count; i++)
        {
            for (unsigned short j = 0; j < 3; j++)
            {
                gyro_sum[i][j] += g_radian_s_sync[i][j];
            }
            counts[i]++;
        }
    }
    for (unsigned short i = 0; i < imu_count; i++)
    {
        for (unsigned short j = 0; j < 3; j++)
        {
            gyro_offs[i][j] = gyro_sum[i][j] / counts[i];
        }
    }
    return true;
}


/*
 * @brief calculate offset and affine transformation for accelerometer
 * @require accel_ref vectors from read average
 * @output accel_T matrixs for affine rotation
 * @output accel_offs vectors for offset calculation
 */
static bool calculate_calibration_values(unsigned short sensor, float g)
{
    for (unsigned short i = 0; i < 3; i++)
    {
        accel_offs[sensor][i] = (accel_ref[sensor][i * 2][i] + accel_ref[sensor][i * 2 + 1][i]) / 2;
    }

    /**
     * fill matrix A for linear equation A * x = b
     * A is the measurement without offset
     * x is the rotation matrix in Affine model
     * b is the reference matrix, [g 0 0; 0 g 0; 0 0 g] = eyes(3) * g
     */
    float mat_A[3][3] = { 0 };

    for (unsigned short i = 0; i < 3; i++)
    {
        for (unsigned short j = 0; j < 3; j++)
        {
            mat_A[i][j] = accel_ref[sensor][i * 2][j] - accel_offs[sensor][j];
        }
    }
    float mat_A_inv[3][3];

    if (!matrix_invert3(mat_A, mat_A_inv))
    {
        return false;
    }

    for (unsigned short i = 0; i < 3; i++)
    {
        for (unsigned short j = 0; j < 3; j++)
        {
            accel_T[sensor][i][j] = mat_A_inv[i][j] * g;
        }
    }
    return true;
}


static void print_accel_output(void)
{
    printf("float accel_offs[imu_count][3]= \n{ \n{ %f,\t%f,\t%f\t },\n \t{ %f,\t%f,\t%f\t },\n}; \n", \
           accel_offs[0][0], accel_offs[0][1], accel_offs[0][2],                                     \
           accel_offs[1][0], accel_offs[1][1], accel_offs[1][2]);
    printf("float accel_T[imu_count][3][3]= \n{ \n{ \n{ %f,\t %f,\t %f\t },\n{%f,\t%f,\t%f\t},\n{ %f,\t%f,\t%f\t},\n},\n{\n{ %f,\t %f,\t %f\t },\n{ %f,\t %f,\t %f\t },\n{ %f,\t %f,\t %f\t},\n},\n};", \
           accel_T[0][0][0], accel_T[0][0][1], accel_T[0][0][2],                                                                                                                                      \
           accel_T[0][1][0], accel_T[0][1][1], accel_T[0][1][2],                                                                                                                                      \
           accel_T[0][2][0], accel_T[0][2][1], accel_T[0][2][2],                                                                                                                                      \
           accel_T[1][0][0], accel_T[1][0][1], accel_T[1][0][2],                                                                                                                                      \
           accel_T[1][1][0], accel_T[1][1][1], accel_T[1][1][2],                                                                                                                                      \
           accel_T[1][2][0], accel_T[1][2][1], accel_T[1][2][2]);
}


static void print_gyro_output(void)
{
    printf("float gyro_offs[imu_count][3]= {\n \t{ %f,\t%f,\t%f\t },\n \t{ %f,\t%f,\t%f\t },\n}; \n", \
           gyro_offs[0][0], gyro_offs[0][1], gyro_offs[0][2],                                         \
           gyro_offs[1][0], gyro_offs[1][1], gyro_offs[1][2]);
}


void calibrate_accelerometer_task(void *arg)
{
    uint32_t task_idx = (uint32_t)arg;
    enum detect_orientation_return orient = DETECT_ORIENTATION_ERROR;
    bool read_avg_result;
    bool calculation_result;

    while (accelerometer_not_calibrated)
    {
        // ESP_LOGI(TAG, "calibration task[%d], state[%d] event:", task_idx, calibration_state_record);
        switch (calibration_state_record)
        {
        case STATE_DETECT_ORIENTATION:
            orient = detect_orientation(task_idx, CONSTANTS_ONE_G);

            side_complete_count = 0;
            for (unsigned short i = 0; i < side_count_max; i++)
            {
                if (side_data_collected[i])
                {
                    side_complete_count++;
                }
            }

            if ((orient == DETECT_ORIENTATION_ERROR))
            {
                detect_attempt++;
                calibration_state_record = (detect_attempt < detect_attempt_max) ? STATE_DETECT_ORIENTATION : STATE_CALIBRATION_ERROR;
            }
            else
            {
                detect_attempt = 0;
                if (side_complete_count == side_count_max)
                {
                    calibration_state_record = STATE_CALCULATION;
                }
                else if (side_data_collected[orient])
                {
                    ESP_LOGI(TAG, "side data [%d] already collected, rotate the board", orient);
                    calibration_state_record = STATE_DETECT_ORIENTATION;
                }
                else
                {
                    calibration_state_record = STATE_READ_AVERAGE;
                }
            }
            break;

        case STATE_READ_AVERAGE:
            read_avg_result = read_accelerometer_avg(orient, 10000);
            if (!read_avg_result)
            {
                ESP_LOGI(TAG, "vibration detected, redo reading average on [%d]", orient);
                calibration_state_record = STATE_READ_AVERAGE;
            }
            else
            {
                side_data_collected[orient] = true;
                ESP_LOGI(TAG, "Orientation[%d] reading average done, %d side finished", orient, side_complete_count);
                calibration_state_record = STATE_DETECT_ORIENTATION;
            }
            break;

        case STATE_CALCULATION:
            for (unsigned short i = 0; i < imu_count; i++)
            {
                calculation_result = calculate_calibration_values(i, CONSTANTS_ONE_G);
                if (!calculation_result)
                {
                    ESP_LOGI(TAG, "accelerometer[%d] has singularity in calibration", i);
                    calibration_state_record = STATE_DETECT_ORIENTATION;
                }
            }
            calibration_state_record = STATE_CALIBRATION_DONE;
            break;

        case STATE_CALIBRATION_DONE:
            accelerometer_not_calibrated = false;
            ESP_LOGI(TAG, "accelerometer calibration task is done.");

            print_accel_output();
            print_gyro_output();

            break;

        case STATE_CALIBRATION_ERROR:
            accelerometer_not_calibrated = false;
            ESP_LOGI(TAG, "calibration error, closed the process.");
            break;

        default:
            break;
        }
        vTaskDelay(200 / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}


void calibrate_gyroscope_task(void *arg)
{
    uint32_t task_idx = (uint32_t)arg;

    while (gyroscope_not_calibrated)
    {
        if ((calibration_state_record == STATE_READ_AVERAGE) && gyroscope_not_calibrated)
        {
            gyroscope_not_calibrated = !read_gyrscope_avg(10000);
            if (!gyroscope_not_calibrated)
            {
                ESP_LOGI(TAG, "gyroscope calibration task[%d] done", task_idx);
                printf("gyro offsets for the first imu:  %f\t%f\t%f\n", gyro_offs[0][0], gyro_offs[0][1], gyro_offs[0][2]);
                printf("gyro offsets for the second imu: %f\t%f\t%f\n", gyro_offs[1][0], gyro_offs[1][1], gyro_offs[1][2]);
            }
        }
        vTaskDelay(200 / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}
