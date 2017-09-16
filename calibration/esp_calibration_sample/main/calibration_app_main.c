/**
20170523 Beck Pang
* @brief seperate calibration main function so the components can be clean and shared
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "calibration.h"
#include "timercmp.h"
#include "matrix_math.h"
#include "observer_app.h"

xSemaphoreHandle  print_mux;

void app_main()
{
    print_mux = xSemaphoreCreateMutex();
    imu_task_init();

    printf("finished initialization for i2c test app.\n");
    xTaskCreate(imu_update_task, "imu_update_task_0", 1024 * 2, (void * )0, 5, NULL);
    xTaskCreate(calibrate_accelerometer_task, "calibrate_task_0", 1024 * 2, (void * )0, 2, NULL);
    xTaskCreate(calibrate_gyroscope_task, "calibrate_task_1", 1024 * 2, (void * )0, 2, NULL);
    xTaskCreate(observer_task, "observer_task_0", 1024 * 2, (void * )3, 6, NULL);
}
