#include "main.h"
#include "shell.h"
#include "calibrate_sensor.h"

#include <string.h>

static PIMUStruct pIMU_1;
static PGyroStruct pGyro;

#define SYNC_SEQ  0xaabbccdd
static void transmit_host
  (BaseSequentialStream* chp,
    uint32_t* const txbuf_d, float* const txbuf_f,
    const uint8_t num_int, const uint8_t num_float)
{
  uint32_t sync = SYNC_SEQ;
  char* byte = (char*)&sync;

  uint8_t i;
  for (i = 0; i < 4; i++)
    chSequentialStreamPut(chp, *byte++);

  byte = (char*)txbuf_d;
  for (i = 0; i < 4*num_int; i++)
    chSequentialStreamPut(chp, *byte++);

  byte = (char*)txbuf_f;
  for (i = 0; i < 4*num_float; i++)
    chSequentialStreamPut(chp, *byte++);
}

#define HOST_TRANSMIT_FREQ  100U
static THD_WORKING_AREA(Host_thread_wa, 512);
static THD_FUNCTION(Host_thread, p)
{
  (void)p;
  chRegSetThreadName("Host tramsmitter");

  if((*SERIAL_DATA).state != SD_READY)
    sdStart(SERIAL_DATA, NULL);

  int32_t txbuf_d[16];
  float txbuf_f[16];
  BaseSequentialStream* chp = (BaseSequentialStream*)SERIAL_DATA;

  uint32_t tick = chVTGetSystemTimeX();

  const uint16_t period = US2ST(1000000/HOST_TRANSMIT_FREQ);
  while (true)
  {
    tick += period;
    if(tick > chVTGetSystemTimeX())
      chThdSleepUntil(tick);
    else
    {
      tft_printf(3,6,"Host thread timeout occured!");
      break;
    }

    txbuf_f[0] = pGyro->angle * 180.0f/M_PI;
    txbuf_f[1] = pGyro->angle_vel * 180.0f/M_PI;

    transmit_host(chp, txbuf_d, txbuf_f, 0, 2);
  }

  while(true)
  {
    palTogglePad(GPIOB,GPIOB_LED);
    chThdSleepMilliseconds(100);
  }
}

static THD_WORKING_AREA(Shell_thread_wa, 1024);
static void cmd_test(BaseSequentialStream * chp, int argc, char *argv[])
{
  chprintf(chp,"gyro_angle:%f\r\n",pGyro->angle * 180.0f/M_PI);
  chprintf(chp,"gyro_angle_vel:%f\r\n",pGyro->angle_vel * 180.0f/M_PI);

  float test;
  flashRead(GYRO_CAL_FLASH, &test,4);
  chprintf(chp,"gyro_offset: %f\r\n",  test * 180.0f/M_PI);
}

static void cmd_data(BaseSequentialStream * chp, int argc, char *argv[])
{
  uint8_t sec = 10;

  if(argc)
  {
    char *toNumber = argv[0];
    uint32_t finalNum=0;
    while(*toNumber>='0' && *toNumber<='9')
      finalNum=finalNum*10+*(toNumber++)-'0';

    if(finalNum == 0)
      finalNum = 10;

    sec = (finalNum < 60 ? finalNum : 60);
  }

  chprintf(chp,"Data transmission start in %d seconds...\r\n", sec);
  chThdSleepSeconds(sec);

  chThdCreateStatic(Host_thread_wa, sizeof(Host_thread_wa),
                    NORMALPRIO,
                    Host_thread, NULL);
}

/*Definition of shell cmd funtions goes here*/
static const ShellCommand commands[] =
{
  {"test", cmd_test},
  {"data", cmd_data},
  {"calimu", cmd_calibrate_imu},
  {"calgyro", cmd_calibrate_gyro}
};

static const ShellConfig shell_cfg1 =
{
  (BaseSequentialStream *)SERIAL_CMD,
  commands
};

void shellStart(void)
{
  pIMU_1 = mpu6050_get();
  pGyro = gyro_get();

  sdStart(SERIAL_CMD, NULL);

  shellInit();
  shellCreateStatic(&shell_cfg1, Shell_thread_wa,
      sizeof(Shell_thread_wa), NORMALPRIO);
}
