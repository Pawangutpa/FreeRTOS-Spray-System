#ifndef RTOS_QUEUES_H
#define RTOS_QUEUES_H

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "common/data_types.h"

/*
 * Queue handle used to pass LiDAR data
 * between LidarTask and other tasks
 */
extern QueueHandle_t g_lidarQueue;
extern QueueHandle_t g_proxiQueue;

#endif