#ifndef PROXI_DRIVER_H
#define PROXI_DRIVER_H

#include <Arduino.h>

/*
 * Initialize proximity + flow sensors
 */
void Proxi_Driver_Init(void);

/*
 * FreeRTOS task for proximity processing
 */
void ProxiTask(void *pvParameters);

#endif