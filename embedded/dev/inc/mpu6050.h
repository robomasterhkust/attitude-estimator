#ifndef _MPU6050_H_
#define _MPU6050_H_

#define MPU6050_UPDATE_FREQ                     100U  //Read MPU @ 50Hz
#define MPuU6050_FLASH_ADDR               0x08040000

typedef enum {
  MPU6050_I2C_ADDR_A0_LOW = 0x68,
  MPU6050_I2C_ADDR_A0_HIGH = 0x69,
} mpu_i2c_addr_t;

typedef enum {
  MPU6050_GYRO_SCALE_250 = 0,
  MPU6050_GYRO_SCALE_500 = 1,
  MPU6050_GYRO_SCALE_1000 = 2,
  MPU6050_GYRO_SCALE_2000 = 3
} mpu_gyro_scale_t;

typedef enum {
  MPU6050_ACCEL_SCALE_2G = 0,
  MPU6050_ACCEL_SCALE_4G = 1,
  MPU6050_ACCEL_SCALE_8G = 2,
  MPU6050_ACCEL_SCALE_16G = 3
} mpu_accel_scale_t;

#define MPU6050_UPDATE_PERIOD     1000000U/MPU6050_UPDATE_FREQ

typedef struct tagIMUStruct {
  float accelData[3];     /* Accelerometer data.             */
  float gyroData[3];      /* Gyroscope data.                 */
  float accelFiltered[3]; /* Filtered accelerometer data.    */
  float gyroFiltered[3];  /* Filtered gyro data.    */
  float accelBias[3];     /* Accelerometer bias.             */
  float gyroBias[3];      /* Gyroscope bias.                 */
  float transAccelBias[3]; /* when pitch >90degree            */
  float v2Filtered[3];    /* Filtered directionattr of gravity.  */
  float qIMU[4];          /* Attitude quaternion of the IMU. */

  I2CDriver* mpu_i2c;
  uint8_t addr;
  float accel_psc;
  float gyro_psc;
} __attribute__((packed)) IMUStruct, *PIMUStruct;

typedef struct {
  I2CDriver* mpu_i2c;
  const mpu_i2c_addr_t a0_high;
  const mpu_accel_scale_t accelConf;
  const mpu_gyro_scale_t gyroConf;
} IMUConfigStruct;

typedef struct {
  int8_t last_i2c_error;
  uint16_t i2c_error_counter;
  uint8_t errorFlag;
} I2CErrorStruct;

#ifdef __cplusplus
extern "C" {
#endif
  PIMUStruct mpu6050_get(void);
  I2CErrorStruct* mpuGetError(void);

  uint8_t mpu6050Init(PIMUStruct pIMU, IMUConfigStruct* imu_conf);
  uint8_t mpu6050GetData(PIMUStruct pIMU);

#ifdef __cplusplus
}
#endif

#endif /* _MPU6050_H_ */
