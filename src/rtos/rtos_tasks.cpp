#include <Arduino.h>
#include "rtos/rtos_tasks.h"
#include "drivers/can_driver.h"
#include "drivers/proxi_driver.h"
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

    g_proxiQueue = xQueueCreate(10, sizeof(proxi_data_t));

Proxi_Driver_Init();

xTaskCreatePinnedToCore(ProxiTask, "ProxiTask", 4096, NULL, 5, NULL, 1);
xTaskCreatePinnedToCore(ProxiDebugTask, "ProxiDebugTask", 4096, NULL, 4, NULL, 1);    
}

QueueHandle_t g_proxiQueue = NULL;

static void ProxiDebugTask(void *pvParameters)
{
    proxi_data_t rx;

    while (true)
    {
        if (xQueueReceive(g_proxiQueue, &rx, portMAX_DELAY))
        {
            Serial.print("Water: ");
            Serial.print(rx.total_water_liters);
            Serial.print(" L | Nut Avg: ");
            Serial.println(rx.nut_count_average);
        }
    }
}