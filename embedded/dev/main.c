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

static THD_WORKING_AREA(TFT_thread_wa, 4096);
static THD_FUNCTION(TFT_thread, p)
{
  (void)p;
  chRegSetThreadName("TFT Display");

  tft_init(TFT_HORIZONTAL, CYAN, BLACK, BLACK);

  tft_printf(3,1,"Robomaster 2018");
  tft_printf(3,2,"TFTLCD");

  while(true)
  {
    tft_update();
    chThdSleepMilliseconds(100);
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

  chThdCreateStatic(TFT_thread_wa, sizeof(TFT_thread_wa),
  NORMALPRIO - 10,
                    TFT_thread, NULL);

  while (true)
  {
    palTogglePad(GPIOB,GPIOB_LED);
    chThdSleepMilliseconds(1000);
  }

  return 0;
}
