/*
 *	20170513, Beck Pang, basic mpu6050 functions
 */
#include "mpu6050.h"
#include "freertos/semphr.h"

xSemaphoreHandle print_mux;


void mpu6500_init(uint8_t devAddr)
{
    uint8_t   i = 0;
    esp_err_t ret;
    uint8_t   MPU6500_Init_Data[8][2] =
    {
        { MPU6500_PWR_MGMT_1,     0x80 }, // Reset Device
        { MPU6500_PWR_MGMT_1,     0x03 }, // Clock Source - Gyro-Z
        { MPU6500_PWR_MGMT_2,     0x00 }, // Enable Acc & Gyro
        { MPU6500_CONFIG,         0x02 }, // LPF 92Hz for gyro and 98Hz for temp
        { MPU6500_GYRO_CONFIG,    0x00 }, // +-256dps
        { MPU6500_ACCEL_CONFIG,   0x00 }, // +-2G
        { MPU6500_ACCEL_CONFIG_2, 0x02 }, // Set Acc LPF, 92Hz
        { MPU6500_USER_CTRL,      0x20 }, // Enable AUX
    };

    for (i = 0; i < 8; i++)
    {
        ret = i2c_write_reg(I2C_MASTER_NUM_0, devAddr, MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
        if (ret != ESP_OK)
        {
            xSemaphoreTake(print_mux, portMAX_DELAY);
            printf("initialization failed on step %d\n", i);
            xSemaphoreGive(print_mux);
        }
    }
}


esp_err_t mpu6500_getMotion6(uint8_t devAddr, imu_data_t *imu_data)
{
    esp_err_t ret;
    uint8_t   *imu_buff = (uint8_t *)malloc(14);

    ret = i2c_read_words(I2C_MASTER_NUM_0, devAddr, MPU6500_ACCEL_XOUT_H, imu_buff, 14);

    if (ret == ESP_OK)
    {
        imu_data->a_raw[0] = imu_buff[0] << 8 | imu_buff[1];
        imu_data->a_raw[1] = imu_buff[2] << 8 | imu_buff[3];
        imu_data->a_raw[2] = imu_buff[4] << 8 | imu_buff[5];

        imu_data->temp = imu_buff[6] << 8 | imu_buff[7];

        imu_data->g_raw[0] = imu_buff[8] << 8 | imu_buff[9];
        imu_data->g_raw[1] = imu_buff[10] << 8 | imu_buff[11];
        imu_data->g_raw[2] = imu_buff[12] << 8 | imu_buff[13];

        imu_data->a_m_s2[0] = (float)imu_data->a_raw[0] * CONSTANTS_ONE_G / accelerometer_sensitivity;
        imu_data->a_m_s2[1] = (float)imu_data->a_raw[1] * CONSTANTS_ONE_G / accelerometer_sensitivity;
        imu_data->a_m_s2[2] = (float)imu_data->a_raw[2] * CONSTANTS_ONE_G / accelerometer_sensitivity;

        imu_data->g_radian_s[0] = (float)imu_data->g_raw[0] * GYRO_TO_RADIAN / GYRO_MAX;
        imu_data->g_radian_s[1] = (float)imu_data->g_raw[1] * GYRO_TO_RADIAN / GYRO_MAX;
        imu_data->g_radian_s[2] = (float)imu_data->g_raw[2] * GYRO_TO_RADIAN / GYRO_MAX;
        // printf("accelerom data: %f\t %f\t %f\n", imu_data->a_m_s2[0], imu_data->a_m_s2[1], imu_data->a_m_s2[2]);
        // printf("gyroscope data: %f\t %f\t %f\n", imu_data->g_radian_s[0], imu_data->g_radian_s[1], imu_data->g_radian_s[2]);
    }
    else
    {
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Error: No ack, sensor[%x] not connected\n", devAddr);
        xSemaphoreGive(print_mux);
    }
    free(imu_buff);
    imu_buff = NULL;
    return ret;
}
