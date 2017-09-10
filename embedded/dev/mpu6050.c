/**
 * This is device realize "read through write" paradigm. This is not
 * standard, but most of I2C devices use this paradigm.
 * You must write to device reading address, send restart to bus,
 * and then begin reading process.
 */

#include "ch.h"
#include "hal.h"

//#include "telemetry.h"
#include "mpu6050.h"
#include "flash.h"

#include "chprintf.h"
/* C libraries: */
#include <string.h>
#include <math.h>

#define MPU_USE_I2C &I2CD1

/* MPU6050 useful registers */
#define MPU6050_SMPLRT_DIV        0x19
#define MPU6050_CONFIG            0x1A
#define MPU6050_GYRO_CONFIG       0x1B
#define MPU6050_ACCEL_CONFIG      0x1C
#define MPU6050_ACCEL_XOUT_H      0x3B
#define MPU6050_ACCEL_XOUT_L      0x3C
#define MPU6050_ACCEL_YOUT_H      0x3D
#define MPU6050_ACCEL_YOUT_L      0x3E
#define MPU6050_ACCEL_ZOUT_H      0x3F
#define MPU6050_ACCEL_ZOUT_L      0x40
#define MPU6050_TEMP_OUT_H        0x41
#define MPU6050_TEMP_OUT_L        0x42
#define MPU6050_GYRO_XOUT_H       0x43
#define MPU6050_GYRO_XOUT_L       0x44
#define MPU6050_GYRO_YOUT_H       0x45
#define MPU6050_GYRO_YOUT_L       0x46
#define MPU6050_GYRO_ZOUT_H       0x47
#define MPU6050_GYRO_ZOUT_L       0x48
#define MPU6050_PWR_MGMT_1        0x6B

/* Sensor scales */
#define MPU6050_GYRO_SCALE        (1.0f / 131.0f) //  250 deg/s
//#define MPU6050_GYRO_SCALE        (1.0f /  65.5f) //  500 deg/s
//#define MPU6050_GYRO_SCALE        (1.0f /  32.8f) // 1000 deg/s
//#define MPU6050_GYRO_SCALE        (1.0f /  16.4f) // 2000 deg/s

#define GRAV                      9.81f
//#define GRAV_BIA   9.81f*
#define MPU6050_ACCEL_SCALE       (GRAV / 16384.0f) //  2G
//#define MPU6050_ACCEL_SCALE       (GRAV /  8192.0f) //  4G
//#define MPU6050_ACCEL_SCALE       (GRAV /  4096.0f) //  8G
//#define MPU6050_ACCEL_SCALE       (GRAV /  2048.0f) // 16G

#define MPU6050_RX_BUF_SIZE       0x0E
#define MPU6050_TX_BUF_SIZE       0x05

#define IMU_AXIS_DIR_POS          0x08
#define IMU_AXIS_ID_MASK          0x07

#define IMU1_AXIS_DIR_POS         0x08
#define IMU1_AXIS_ID_MASK         0x07
#define IMU1_CONF_MASK            0x0F

#define IMU2_AXIS_DIR_POS         0x80
#define IMU2_AXIS_ID_MASK         0x70
#define IMU2_CONF_MASK            0xF0

/* I2C read transaction time-out in milliseconds. */
#define MPU6050_READ_TIMEOUT_MS   0x01
/* I2C write transaction time-out in milliseconds. */
#define MPU6050_WRITE_TIMEOUT_MS  0x01

#define CALIBRATION_COUNTER_MAX   2000

/**
 * Global variables
 */
/* Default packed sensor settings.
 * Structure of the sensor settings is:
 * D2|2I2|1I2|0I2|D1|2I1|1I1|0I1
 * where Dx  - axis direction of the sensor x;
 *       nIx - n-th bit of axis ID of the sensor x.
 *         |
  IMU1_AXIS_DIR_POS
 */
uint8_t g_sensorSettings[3] = {
  0x22  ,              /* Pitch (X) */
  0x00|
  IMU1_AXIS_DIR_POS ,              /* Roll  (Y) */
  0x11
/* Yaw   (Z) */
};

static const I2CConfig i2cfg = {
    OPMODE_I2C,
    200000,
    FAST_DUTY_CYCLE_2
};

/* IMU data structure. */
IMUStruct g_IMU1;

/* Flash stored structure */
//MPUFlashStruct MPUFlash;
float MPUFlash[6];

/* I2C error info structure. */
I2CErrorStruct g_i2cErrorInfo = {0, 0, 0};

extern uint32_t g_boardStatus;

/**
 * Local variables
 */
/* Data buffers */
static int16_t mpu6050Data[6];

static uint8_t mpu6050RXData[MPU6050_RX_BUF_SIZE];
static uint8_t mpu6050TXData[MPU6050_TX_BUF_SIZE];

int16_t* mpuGetData(void)
{
  return mpu6050Data;
}

I2CErrorStruct* mpuGetError(void)
{
  return &g_i2cErrorInfo;
}

/**
 * @brief  Initialization function of IMU data structure.
 * @param  pIMU - pointer to IMU data structure;
 * @param  fAddrLow - IMU address pin A0 is pulled low flag.
 */
void imuStructureInit(PIMUStruct pIMU, uint8_t fAddrHigh) {
  uint8_t i;
  /* Initialize to zero. */
  memset((void *)pIMU, 0, sizeof(IMUStruct));
  memset(MPUFlash, 0, 24);

  flashRead(MPU_FLASH_ADDR,(char*)MPUFlash, 24);
  for (i = 0; i < 3U; i++)
  {
     pIMU->accelBias[i] = MPUFlash[i];
     pIMU->gyroBias[i] = MPUFlash[i + 3];
  }

  if (fAddrHigh) {
    pIMU->addr = MPU6050_I2C_ADDR_A0_HIGH;
    for (i = 0; i < 3; i++) {
      pIMU->axes_conf[i] = g_sensorSettings[i] >> 4;
    }
  } else {
    pIMU->addr = MPU6050_I2C_ADDR_A0_LOW;
    for (i = 0; i < 3; i++) {
      pIMU->axes_conf[i] = g_sensorSettings[i] & IMU1_CONF_MASK;
    }
  }
}

/**
 * @brief
 */
void imuCalibrationSet(uint8_t flags) {
  g_boardStatus |= flags & IMU_CALIBRATION_MASK;
}

/**
 * @brief  Initialization function of IMU data structure.
 * @param  pIMU - pointer to IMU data structure;
 * @param  fCalibrateAcc - accelerometer calibration flag.
 * @return 0 - if calibration is not finished;
 *         1 - if calibration is finished.
 */
int8_t imuCalibrate(PIMUStruct pIMU, uint8_t fCalibrateAcc) {
  if(!mpu6050GetNewData(&g_IMU1))
    return -1;

  if (fCalibrateAcc) {
    if (pIMU->clbrCounter == 0) {
      /* Reset accelerometer bias. */
      pIMU->accelBias[0] = pIMU->accelData[0] + GRAV;
      pIMU->accelBias[1] = pIMU->accelData[1];
      pIMU->accelBias[2] = pIMU->accelData[2];
      pIMU->clbrCounter++;
      return 0;
    } else if (pIMU->clbrCounter < CALIBRATION_COUNTER_MAX) {
      /* Accumulate accelerometer bias. */
      pIMU->accelBias[0] += pIMU->accelData[0] + GRAV;
      pIMU->accelBias[1] += pIMU->accelData[1];
      pIMU->accelBias[2] += pIMU->accelData[2];
      pIMU->clbrCounter++;
      return 0;
    } else {
      /* Update accelerometer bias. */
      pIMU->accelBias[0] /= (float)CALIBRATION_COUNTER_MAX;
      pIMU->accelBias[1] /= (float)CALIBRATION_COUNTER_MAX;
      pIMU->accelBias[2] /= (float)CALIBRATION_COUNTER_MAX;
      pIMU->clbrCounter = 0;
    }
  } else {
    if (pIMU->clbrCounter == 0) {
      /* Reset gyroscope bias. */
      pIMU->gyroBias[0] = pIMU->gyroData[0];
      pIMU->gyroBias[1] = pIMU->gyroData[1];
      pIMU->gyroBias[2] = pIMU->gyroData[2];
      pIMU->clbrCounter++;
      return 0;
    } else if (pIMU->clbrCounter < CALIBRATION_COUNTER_MAX) {
      /* Accumulate gyroscope bias. */
      pIMU->gyroBias[0] += pIMU->gyroData[0];
      pIMU->gyroBias[1] += pIMU->gyroData[1];
      pIMU->gyroBias[2] += pIMU->gyroData[2];
      pIMU->clbrCounter++;
      return 0;
    } else {
      /* Update gyroscope bias. */
      pIMU->gyroBias[0] /= (float)CALIBRATION_COUNTER_MAX;
      pIMU->gyroBias[1] /= (float)CALIBRATION_COUNTER_MAX;
      pIMU->gyroBias[2] /= (float)CALIBRATION_COUNTER_MAX;
      pIMU->clbrCounter = 0;
    }
  }
  return 1;
}

/**
 * @brief  Initialization function for the MPU6050 sensor.
 * @param  addr - I2C address of MPU6050 chip.
 * @return 1 - if initialization was successful;
 *         0 - if initialization failed.
 */
uint8_t mpu6050Init(uint8_t addr) {
  msg_t status = MSG_OK;

  i2cStart(MPU_USE_I2C, &i2cfg);

  /* Reset all MPU6050 registers to their default values */
  mpu6050TXData[0] = MPU6050_PWR_MGMT_1;  // Start register address;
  mpu6050TXData[1] = 0xC0;          // Register value 0b11000000

  i2cAcquireBus(MPU_USE_I2C);

  status = i2cMasterTransmitTimeout(MPU_USE_I2C, addr, mpu6050TXData, 2,
    NULL, 0, MS2ST(MPU6050_WRITE_TIMEOUT_MS));

  if (status != MSG_OK) {
    i2cReleaseBus(MPU_USE_I2C);
    g_i2cErrorInfo.last_i2c_error = i2cGetErrors(MPU_USE_I2C);
    if (g_i2cErrorInfo.last_i2c_error) {
      g_i2cErrorInfo.i2c_error_counter++;
    }
    g_i2cErrorInfo.errorFlag |= 0x02;
    return 0;
  }

  /* Wait 100 ms for the MPU6050 to reset */
  chThdSleepMilliseconds(100);

  /* Clear the SLEEP flag, set the clock and start measuring. */
  mpu6050TXData[0] = MPU6050_PWR_MGMT_1;  // Start register address;
  mpu6050TXData[1] = 0x03;         // Register value CLKSEL = PLL_Z;

  status = i2cMasterTransmitTimeout(MPU_USE_I2C, addr, mpu6050TXData, 2,
    NULL, 0, MS2ST(MPU6050_WRITE_TIMEOUT_MS));

  if (status != MSG_OK) {
    i2cReleaseBus(MPU_USE_I2C);
    g_i2cErrorInfo.last_i2c_error = i2cGetErrors(MPU_USE_I2C);
    if (g_i2cErrorInfo.last_i2c_error) {
      g_i2cErrorInfo.i2c_error_counter++;
//      debugLog("E:mpu6050i-rst");
    }
    g_i2cErrorInfo.errorFlag |= 0x04;
    return 0;
  }

  /* Configure the MPU6050 sensor        */
  /* NOTE:                               */
  /* - SLEEP flag must be cleared before */
  /*   configuring the sensor.           */
  mpu6050TXData[0] = MPU6050_SMPLRT_DIV;  // Start register address;
  mpu6050TXData[1] = 11;                  // SMPLRT_DIV register value (8000 / (11 + 1) = 666 Hz);
  mpu6050TXData[2] = 0x00;          // CONFIG register value DLPF_CFG = 0 (256-260 Hz);
  mpu6050TXData[3] = 0x00;          // GYRO_CONFIG register value FS_SEL = +-250 deg/s;
  mpu6050TXData[4] = 0x01;          // ACCEL_CONFIG register value AFS_SEL = +-4G

  status = i2cMasterTransmitTimeout(MPU_USE_I2C, addr, mpu6050TXData, 5,
    NULL, 0, MS2ST(MPU6050_WRITE_TIMEOUT_MS));


  i2cReleaseBus(MPU_USE_I2C);

  if (status != MSG_OK) {
    g_i2cErrorInfo.last_i2c_error = i2cGetErrors(MPU_USE_I2C);
    if (g_i2cErrorInfo.last_i2c_error) {
      g_i2cErrorInfo.i2c_error_counter++;
     // debugLog("E:mpu6050i-cfg");
    }
    g_i2cErrorInfo.errorFlag |= 0x08;
    return 0;
  }

  return 1;
}

int mpu6050GetRawData(PIMUStruct pIMU,int16_t mpu6050Data[6]) {
  msg_t status = MSG_OK;

  /* Set the start register address for bulk data transfer. */
  mpu6050TXData[0] = MPU6050_ACCEL_XOUT_H;

  //palTogglePad(GPIOD,GPIOD_LED4);

  i2cAcquireBus(MPU_USE_I2C);
  status = i2cMasterTransmitTimeout(MPU_USE_I2C, pIMU->addr, mpu6050TXData, 1,
    mpu6050RXData, 14, MS2ST(MPU6050_READ_TIMEOUT_MS));
	i2cReleaseBus(MPU_USE_I2C);

  //palTogglePad(GPIOD,GPIOD_LED3);

  if (status != MSG_OK) {
    g_i2cErrorInfo.last_i2c_error = i2cGetErrors(MPU_USE_I2C);
    if (g_i2cErrorInfo.last_i2c_error) {
      g_i2cErrorInfo.i2c_error_counter++;
 //     debugLog("E:mpu6050gnd");
    }
    return -1;
  }

  mpu6050Data[0] = (int16_t)((mpu6050RXData[ 0]<<8) | mpu6050RXData[ 1]); /* Accel X */
  mpu6050Data[1] = (int16_t)((mpu6050RXData[ 2]<<8) | mpu6050RXData[ 3]); /* Accel Y */
  mpu6050Data[2] = (int16_t)((mpu6050RXData[ 4]<<8) | mpu6050RXData[ 5]); /* Accel Z */
  mpu6050Data[3] = (int16_t)((mpu6050RXData[ 8]<<8) | mpu6050RXData[ 9]); /* Gyro X  */
  mpu6050Data[4] = (int16_t)((mpu6050RXData[10]<<8) | mpu6050RXData[11]); /* Gyro Y  */
  mpu6050Data[5] = (int16_t)((mpu6050RXData[12]<<8) | mpu6050RXData[13]); /* Gyro Z  */
  return 0;
}


/**
 * @brief  Reads new data from the sensor
 * @param  pIMU - pointer to IMU data structure;
 * @return 1 - if reading was successful;
 *         0 - if reading failed.
 */
uint8_t mpu6050GetNewData(PIMUStruct pIMU) {
  msg_t status = MSG_OK;
  uint8_t id;
  int16_t mpu6050Data[6];

  /* Set the start register address for bulk data transfer. */
  mpu6050TXData[0] = MPU6050_ACCEL_XOUT_H;
  i2cAcquireBus(MPU_USE_I2C);
  status = i2cMasterTransmitTimeout(MPU_USE_I2C, pIMU->addr, mpu6050TXData, 1,
    mpu6050RXData, 14, MS2ST(MPU6050_READ_TIMEOUT_MS));
	i2cReleaseBus(MPU_USE_I2C);

  if (status != MSG_OK) {
    g_i2cErrorInfo.last_i2c_error = i2cGetErrors(MPU_USE_I2C);
    if (g_i2cErrorInfo.last_i2c_error) {
      g_i2cErrorInfo.i2c_error_counter++;
 //     debugLog("E:mpu6050gnd");
    }
    return 0;
  }

  mpu6050Data[0] = (int16_t)((mpu6050RXData[ 0]<<8) | mpu6050RXData[ 1]); /* Accel X */
  mpu6050Data[1] = (int16_t)((mpu6050RXData[ 2]<<8) | mpu6050RXData[ 3]); /* Accel Y */
  mpu6050Data[2] = (int16_t)((mpu6050RXData[ 4]<<8) | mpu6050RXData[ 5]); /* Accel Z */
  mpu6050Data[3] = (int16_t)((mpu6050RXData[ 8]<<8) | mpu6050RXData[ 9]); /* Gyro X  */
  mpu6050Data[4] = (int16_t)((mpu6050RXData[10]<<8) | mpu6050RXData[11]); /* Gyro Y  */
  mpu6050Data[5] = (int16_t)((mpu6050RXData[12]<<8) | mpu6050RXData[13]); /* Gyro Z  */

  /* Pitch: */
  id = pIMU->axes_conf[0] & IMU_AXIS_ID_MASK;
  if (pIMU->axes_conf[0] & IMU_AXIS_DIR_POS) {
    pIMU->accelData[0] = mpu6050Data[id + 0]*MPU6050_ACCEL_SCALE;
    pIMU->gyroData[0]  = mpu6050Data[id + 3]*MPU6050_GYRO_SCALE;
  } else {
    pIMU->accelData[0] = (-1 - mpu6050Data[id + 0])*MPU6050_ACCEL_SCALE;
    pIMU->gyroData[0]  = (-1 - mpu6050Data[id + 3])*MPU6050_GYRO_SCALE;
  }

  /* Roll: */
  id = pIMU->axes_conf[1] & IMU_AXIS_ID_MASK;
  if (pIMU->axes_conf[1] & IMU_AXIS_DIR_POS) {
    pIMU->accelData[1] = mpu6050Data[id + 0]*MPU6050_ACCEL_SCALE;
    pIMU->gyroData[1]  = mpu6050Data[id + 3]*MPU6050_GYRO_SCALE;
  } else {
    pIMU->accelData[1] = (-1 - mpu6050Data[id + 0])*MPU6050_ACCEL_SCALE;
    pIMU->gyroData[1]  = (-1 - mpu6050Data[id + 3])*MPU6050_GYRO_SCALE;
  }

  /* Yaw: */
  id = pIMU->axes_conf[2] & IMU_AXIS_ID_MASK;
  if (pIMU->axes_conf[2] & IMU_AXIS_DIR_POS) {
    pIMU->accelData[2] = mpu6050Data[id + 0]*MPU6050_ACCEL_SCALE;
    pIMU->gyroData[2]  = mpu6050Data[id + 3]*MPU6050_GYRO_SCALE;
  } else {
    pIMU->accelData[2] = (-1 - mpu6050Data[id + 0])*MPU6050_ACCEL_SCALE;
    pIMU->gyroData[2]  = (-1 - mpu6050Data[id + 3])*MPU6050_GYRO_SCALE;
  }

  return 1;
}
