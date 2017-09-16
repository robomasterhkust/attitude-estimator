/**
 * 20170514, Beck Pang
 * low level imu task
 */

#include "imu_task.h"


float accel_offs[imu_count][3] =
{
    { 0.563781, 1.144199, -0.721514 },
    { 0.432455, 0.942875, -0.551948 },
};
float accel_T[imu_count][3][3] =
{
    {
        {  1.002771, -0.031028,  0.035838 },
        {  0.029882,  0.988719, -0.010681 },
        { -0.034797,  0.014299,  1.000051 },
    },
    {
        {  0.998319, -0.031990,  0.038181 },
        {  0.028757,  0.984635, -0.012098 },
        { -0.030265, -0.011889,  0.999233 },
    },
};
float gyro_offs[imu_count][3] =
{
    { -0.020377,  0.012796, 0.013373 },
    { -0.020974, -0.005791, 0.026810 },
};

/**
 * @brief Physical design pramaters that need to get updated for every board
 * @description 20170523 board on the xz frame tilt 45 degree transfer to body frame
 */
static void transfer_coordinate_from_imu_to_body_frame(float *input_raw, float *output_sync)
{
    /*
     * R_imu_to_body = Rx(-pitch) * Rz(-roll)
     * a_body = R_imu_to_body * a_measurement
     */
    float z_roll  = IMU_Z_ROLL_DEG * M_DEG_TO_RAD;
    float x_pitch = IMU_X_PITCH_DEG * M_DEG_TO_RAD;
    float z_mat[9];
    float x_mat[9];
    float R_mat[9];

    rotz(-z_roll, z_mat);
    rotx(-x_pitch, x_mat);

    matrix_multiply(x_mat, z_mat, 3, 3, 3, R_mat);
    matrix_multiply(R_mat, input_raw, 3, 3, 1, output_sync);
}


static void transfer_accel_offset(float *input_sync, float *output_act, float *offs_ptr, float *T_ptr)
{
    float a_m_s2_temp[3];

    matrix_subtract(input_sync, offs_ptr, 3, 1, a_m_s2_temp);
    matrix_multiply(T_ptr, a_m_s2_temp, 3, 3, 1, output_act);
}


static void transfer_gyro_offset(float *input_sync, float *output_act, float *offs_ptr)
{
    matrix_subtract(input_sync, offs_ptr, 3, 1, output_act);
}


void imu_update_task(void *arg)
{
    // uint32_t task_idx = (uint32_t)arg;

    while (1)
    {
        mpu6050_getMotion6(MPU6500_ADDRESS_AD0_LOW, &imu_data[0]);
        mpu6050_getMotion6(MPU6500_ADDRESS_AD0_HIGH, &imu_data[1]);
        // mpu6500_getMotion6(MPU6500_ADDRESS_AD0_LOW, &imu_data[0]);
        // mpu6500_getMotion6(MPU6500_ADDRESS_AD0_HIGH, &imu_data[1]);

        for (unsigned short i = 0; i < imu_count; i++)
        {
            for (unsigned short j = 0; j < 3; j++)
            {
                a_m_s2_raw[i][j]     = imu_data[i].a_m_s2[j];
                g_radian_s_raw[i][j] = imu_data[i].g_radian_s[j];
            }
            float *a_raw_ptr      = &a_m_s2_raw[i][0];
            float *a_sync_ptr     = &a_m_s2_sync[i][0];
            float *a_act_ptr      = &a_m_s2_act[i][0];
            float *accel_offs_ptr = &accel_offs[i][0];
            float *accel_T_ptr    = &accel_T[i][0][0];
            transfer_coordinate_from_imu_to_body_frame(a_raw_ptr, a_sync_ptr);
            transfer_accel_offset(a_sync_ptr, a_act_ptr, accel_offs_ptr, accel_T_ptr);

            float *g_raw_ptr     = &g_radian_s_raw[i][0];
            float *g_sync_ptr    = &g_radian_s_sync[i][0];
            float *g_act_ptr     = &g_radian_s_act[i][0];
            float *gyro_offs_ptr = &gyro_offs[i][0];
            transfer_coordinate_from_imu_to_body_frame(g_raw_ptr, g_sync_ptr);
            transfer_gyro_offset(g_sync_ptr, g_act_ptr, gyro_offs_ptr);
        }

        vTaskDelay(T_period * 1000 / portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}


/**
 * @brief switch to read external MPU6500 instead
 */
void imu_task_init(void)
{
    i2c_master_init();
    // i2c_master_init_0();
    vTaskDelay(300 / portTICK_RATE_MS);
    mpu6050_init(0x68);
    mpu6050_init(0x69);
    // mpu6500_init(0x68);
    // mpu6500_init(0x69);
}
