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

static PIMUStruct pIMU_1;
static const IMUConfigStruct imu1_conf = {&I2CD1, MPU6050_I2C_ADDR_A0_LOW,
   MPU6050_ACCEL_SCALE_8G, MPU6050_GYRO_SCALE_1000};

static THD_WORKING_AREA(IMU_thread_wa, 4096);
static THD_FUNCTION(IMU_thread, p)
{
  (void)p;
  chRegSetThreadName("IMU Attitude Estimator");
  uint8_t errorCode;

  pIMU_1 = mpu6050_get();

  chThdSleepMilliseconds(100);
  errorCode = attitude_imu_init(pIMU_1, &imu1_conf);

  while(errorCode)
  {
    tft_printf(1,1,"IMU Init Failed: %d", errorCode);
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

    errorCode = attitude_update(pIMU_1);
    if(errorCode)
    {
      tft_clear();
      tft_printf(1,1,"IMU Reading Error %d", errorCode);
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

  shellStart();

  tft_init(TFT_HORIZONTAL, CYAN, BLACK, BLACK);

  chThdCreateStatic(IMU_thread_wa, sizeof(IMU_thread_wa),
  NORMALPRIO + 5,
                    IMU_thread, NULL);

  while (true)
  {
    tft_printf(1,1,"Roll: %4d", (int16_t)(pIMU_1->euler_angle[ROLL] * 180.0f/M_PI));
    tft_printf(1,2,"Pitch:%4d", (int16_t)(pIMU_1->euler_angle[PITCH] * 180.0f/M_PI));
    tft_printf(1,3,"Yaw:  %4d", (int16_t)(pIMU_1->euler_angle[YAW] * 180.0f/M_PI));
    tft_printf(1,4,"DT:   %4d", (int16_t)(pIMU_1->dt * 1000000));
    chThdSleepMilliseconds(50);
  }

  return 0;
}
