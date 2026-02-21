#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include <Arduino.h>
#include "driver/twai.h"

/*
 * Initialize TWAI (CAN) driver
 */
void CAN_Driver_Init(void);

/*
 * FreeRTOS Task:
 * Reads LiDAR frames from CAN bus
 */
void LidarTask(void *pvParameters);

#endif