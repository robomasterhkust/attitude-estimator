/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
#include "main.h"

PIMUStruct pIMU_1;
IMUConfigStruct imu1_conf = {&I2CD1, MPU6050_I2C_ADDR_A0_LOW,
   MPU6050_ACCEL_SCALE_8G, MPU6050_GYRO_SCALE_1000};

static THD_WORKING_AREA(IMU_thread_wa, 4096);
static THD_FUNCTION(IMU_thread, p)
{
  (void)p;
  chRegSetThreadName("IMU Attitude Estimator");
  uint8_t init_error;

  pIMU_1 = mpu6050_get();

  chThdSleepMilliseconds(100);
  init_error = mpu6050Init(pIMU_1, &imu1_conf);

  while(init_error)
  {
    tft_printf(1,1,"IMU Init Failed: %d", init_error);
    chThdSleepMilliseconds(500);
  }

  uint32_t tick = chVTGetSystemTimeX();
  while(true)
  {
    tick += US2ST(MPU6050_UPDATE_PERIOD);
    if(chVTGetSystemTimeX() < tick)
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
    }

    if(!mpu6050GetData(pIMU_1))
    {
      tft_printf(1,1,"AccelX:%8d", (int16_t)(pIMU_1->accelData[0]));
      tft_printf(1,2,"AccelY:%8d", (int16_t)(pIMU_1->accelData[1]));
      tft_printf(1,3,"AccelZ:%8d", (int16_t)(pIMU_1->accelData[2]));
      tft_printf(1,4,"GyroX :%8d", (int16_t)(pIMU_1->gyroData[0]));
      tft_printf(1,5,"GyroY :%8d", (int16_t)(pIMU_1->gyroData[1]));
      tft_printf(1,6,"GyroZ :%8d", (int16_t)(pIMU_1->gyroData[2]));
    }
    else
    {
      tft_clear();
      tft_printf(1,1,"IMU Reading Error!");
    }
  }
}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  //shellStart();

  tft_init(TFT_HORIZONTAL, CYAN, BLACK, BLACK);

  chThdCreateStatic(IMU_thread_wa, sizeof(IMU_thread_wa),
  NORMALPRIO + 5,
                    IMU_thread, NULL);

  while (true)
  {
    palTogglePad(GPIOB,GPIOB_LED);
    chThdSleepMilliseconds(500);
  }

  return 0;
}
