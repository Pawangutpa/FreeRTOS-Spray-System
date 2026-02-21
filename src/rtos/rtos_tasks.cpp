#include <Arduino.h>
#include "rtos/rtos_tasks.h"
#include "drivers/can_driver.h"
#include "rtos/rtos_queues.h"
#include "common/data_types.h"

/* ==============================
   Queue Definitions
   ============================== */

QueueHandle_t g_lidarQueue = NULL;

/* ==============================
   Debug Task (Print LiDAR)
   ============================== */

static void LidarDebugTask(void *pvParameters)
{
    lidar_data_t received_data;

    while (true)
    {
        if (xQueueReceive(g_lidarQueue, &received_data, portMAX_DELAY))
        {
            Serial.print("LEFT: ");
            Serial.print(received_data.left_distance_cm);
            Serial.print(" cm  |  RIGHT: ");
            Serial.println(received_data.right_distance_cm);
        }
    }
}

/* ==============================
   CREATE ALL TASKS
   ============================== */

void RTOS_CreateTasks(void)
{
    /*
     * Create queue for LiDAR data
     * Length = 10 items
     */
    g_lidarQueue = xQueueCreate(10, sizeof(lidar_data_t));

    /*
     * Create Lidar Task (Core 1)
     */
    xTaskCreatePinnedToCore(
        LidarTask,
        "LidarTask",
        4096,
        NULL,
        5,
        NULL,
        1);

    /*
     * Create Debug Print Task (Core 1)
     */
    xTaskCreatePinnedToCore(
        LidarDebugTask,
        "LidarDebugTask",
        4096,
        NULL,
        4,
        NULL,
        1);
}