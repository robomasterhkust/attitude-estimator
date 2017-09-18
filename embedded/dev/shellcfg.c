#include "main.h"
#include "shell.h"

#include <string.h>

static PIMUStruct pIMU_1;

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

    txbuf_f[0] = pIMU_1->euler_angle[ROLL];
    txbuf_f[1] = pIMU_1->euler_angle[PITCH];
    txbuf_f[2] = pIMU_1->euler_angle[YAW];

    transmit_host(chp, txbuf_d, txbuf_f, 0, 3);
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
  float data[3] = {0.0f,0.0f,0.0f};

  float flash_test;

  if(argc)
  {
    flashRead(IMU_CAL_FLASH, &flash_test, 4);
    if(isfinite(flash_test))
    {
      flashRead(IMU_CAL_FLASH, (char*)(pIMU_1->accelBias), 48);
      flashRead(IMU_CAL_FLASH + 48, (char*)data, 12);
    }

    data[0] = -data[0];
    data[1] = -data[1];
    data[2] = -data[2];

    flashSectorErase(flashSectorAt(IMU_CAL_FLASH));
    flashWrite(IMU_CAL_FLASH, (char*)(pIMU_1->accelBias), 48);
    flashWrite(IMU_CAL_FLASH + 48, (char*)data, 12);
  }

  flashRead(IMU_CAL_FLASH, &flash_test, 4);
  if(isfinite(flash_test))
  {
    flashRead(IMU_CAL_FLASH, (char*)(pIMU_1->accelBias), 48);
    flashRead(IMU_CAL_FLASH + 48, (char*)data, 12);
  }

  chprintf(chp,"%f\r\n",pIMU_1->accelBias[X]);
  chprintf(chp,"%f\r\n",pIMU_1->accelBias[Y]);
  chprintf(chp,"%f\r\n",pIMU_1->accelBias[Z]);

  chprintf(chp,"%f,%f,%f\r\n",
    pIMU_1->accelT[X][X],pIMU_1->accelT[X][Y],pIMU_1->accelT[X][Z]);
  chprintf(chp,"%f,%f,%f\r\n",
    pIMU_1->accelT[Y][X],pIMU_1->accelT[Y][Y],pIMU_1->accelT[Y][Z]);
  chprintf(chp,"%f,%f,%f\r\n",
    pIMU_1->accelT[Z][X],pIMU_1->accelT[Z][Y],pIMU_1->accelT[Z][Z]);

  chprintf(chp,"%f\r\n",data[0]);
  chprintf(chp,"%f\r\n",data[1]);
  chprintf(chp,"%f\r\n",data[2]);
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

  chprintf(chp,"Data transmission start in %d seconds...\n", sec);
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
  {"cal", cmd_calibration}
};

static const ShellConfig shell_cfg1 =
{
  (BaseSequentialStream *)SERIAL_CMD,
  commands
};

void shellStart(void)
{
  pIMU_1 = mpu6050_get();

  sdStart(SERIAL_CMD, NULL);

  shellInit();
  shellCreateStatic(&shell_cfg1, Shell_thread_wa,
      sizeof(Shell_thread_wa), NORMALPRIO);
}
