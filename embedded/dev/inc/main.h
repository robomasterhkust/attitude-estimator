#ifndef _MAIN_H_
#define _MAIN_H_

#include "ch.h"
#include "hal.h"

#include "flash.h"
#include "mpu6050.h"
#include "chprintf.h"
#include "tft_display.h"
#include "math_misc.h"

#define SERIAL_CMD       &SD3
#define SERIAL_DATA      &SD3

/*
static inline void chprintfloat(BaseSequentialStream* chp, const float f)
{
  uint8_t i, sig = 0;

  if(f < 0.0f)
  {
    chprintf("-");
    f = -f;
  }

  uint8_t dec = (uint8_t)f;

  chprintf(chp,"%d",dec);
  if(dec)
    sig = 1;

  f -= (float)dec;
  f *= 10.0f;

  if(f == 0.0f)
    return;
  else
    chprintf(chp,".");

  for (i = 0; i < 11U; i++)
  {
    f -= (float)dec;
    f *= 10.0f;
    dec = (uint8_t)f;

    if(dec || sig)
      sig++;
    if(sig > 3 || f == 0.0f)
      return;

    chprintf(chp,"%d",dec);
  }
}
*/

void shellStart(void);

#endif
