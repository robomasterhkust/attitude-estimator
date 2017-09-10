#ifndef _MPU6050_H_
#define _MPU6050_H_

#define MPU_FREQ                   400U  //Read MPU @ 50Hz
#define MPU_COUNT     1000000U/MPU_FREQ

#define MPU6050_I2C_ADDR_A0_LOW   0x68 //todo:modify
#define MPU6050_I2C_ADDR_A0_HIGH  0x69

#define MPU_FLASH_ADDR 0x08040000

#define IMU1_CALIBRATE_GYRO         0x00000008
#define IMU1_CALIBRATE_ACCEL        0x00000010
#define IMU1_CALIBRATION_MASK       0x00000018
//#define IMU2_CALIBRATE_GYRO         0x00000020
//#define IMU2_CALIBRATE_ACCEL        0x00000040
//#define IMU2_CALIBRATION_MASK       0x00000060
#define IMU2_CALIBRATE_GYRO         0x00000020
#define IMU2_CALIBRATE_ACCEL        0x00000040
#define IMU2_CALIBRATION_MASK       0x00000060
#define MOTORCHANGED                0x00000200
#define IMU_CALIBRATION_MASK        0x00000078

typedef struct tagIMUStruct {
  float accelData[3];     /* Accelerometer data.             */
  float gyroData[3];      /* Gyroscope data.                 */
  float accumAccelErr[3];
  float gyroFiltered[3];
  float accelBias[3];     /* Accelerometer bias.             */
  float gyroBias[3];      /* Gyroscope bias.                 */
  float transAccelBias[3]; /* when pitch >90degree            */
  float accelFiltered[3]; /* Filtered accelerometer data.    */
  float v2Filtered[3];    /* Filtered directionattr of gravity.  */
  //float qIMU[4];          /* Attitude quaternion of the IMU. */
  /* Angle of leaning.       */
  float theta_accl;
  float theta_gyro;
  float theta;

  uint32_t clbrCounter;   /* Calibration counter             */
  uint8_t axes_conf[3];   /* Configuration of IMU axes.      */
  uint8_t addr;           /* I2C address of the chip.        */
  uint8_t accelFlag;      /* help to decide which bias used  */
} __attribute__((packed)) IMUStruct, *PIMUStruct;

/*
typedef struct flashStoredMPU {
  float gyroBiasFlash[3];
  float accelBiasFlash[3];
} __attribute__((packed)) MPUFlashStruct, *PMPUFlashStruct;*/

typedef struct {
  int8_t last_i2c_error;
  uint16_t i2c_error_counter;
  uint8_t errorFlag;
} I2CErrorStruct;

extern float MPUFlash[6];

/* IMU data structure. */
extern IMUStruct g_IMU1;
/* Packed sensor settings. */
extern uint8_t g_sensorSettings[3];

#ifdef __cplusplus
extern "C" {
#endif

  int16_t* mpuGetData(void);
  I2CErrorStruct* mpuGetError(void);

  void imuStructureInit(PIMUStruct pIMU, uint8_t fAddrHigh);
  void imuCalibrationSet(uint8_t flags);
  int8_t imuCalibrate(PIMUStruct pIMU, uint8_t fCalibrateAcc);
  uint8_t mpu6050Init(uint8_t addr);
  uint8_t mpu6050GetNewData(PIMUStruct pIMU);
  void initWithAccData(PIMUStruct self);
  void mpu6050update(PIMUStruct pIMU);
  void mpu_calc_theta(PIMUStruct pIMU);
#ifdef __cplusplus
}
#endif

#endif /* _MPU6050_H_ */
