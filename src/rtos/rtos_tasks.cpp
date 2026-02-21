#include <Arduino.h>
#include "rtos/rtos_tasks.h"
#include "rtos/rtos_queues.h"
#include "rtos/rtos_events.h"
#include "drivers/can_driver.h"
#include "drivers/proxi_driver.h"
#include "drivers/button_driver.h"
#include "common/data_types.h"
#include "services/spray_service.h"
/* ==============================
   GLOBAL OBJECTS
   ============================== */
QueueHandle_t g_lidarQueue = NULL;
QueueHandle_t g_proxiQueue = NULL;
EventGroupHandle_t g_systemEventGroup = NULL;

/* ==============================
   DEBUG TASKS
   ============================== */
static void LidarDebugTask(void* pvParameters)
{
    lidar_data_t rx;
    while (true)
    {
        if (xQueueReceive(g_lidarQueue, &rx, portMAX_DELAY))
        {
            Serial.print("LIDAR L:");
            Serial.print(rx.left_distance_cm);
            Serial.print(" R:");
            Serial.println(rx.right_distance_cm);
        }
    }
}

static void ProxiDebugTask(void* pvParameters)
{
    proxi_data_t rx;
    while (true)
    {
        if (xQueueReceive(g_proxiQueue, &rx, portMAX_DELAY))
        {
            Serial.print("Water:");
            Serial.print(rx.total_water_liters);
            Serial.print(" | NutAvg:");
            Serial.println(rx.nut_count_average);
        }
    }
}

static void ModeMonitorTask(void* pvParameters)
{
    while (true)
    {
        EventBits_t bits =
            xEventGroupGetBits(g_systemEventGroup);

        if (bits & SYSTEM_BIT_AUTONOMOUS)
        {
            Serial.println("System Running in AUTONOMOUS Mode");
        }
        else if (bits & SYSTEM_BIT_MANUAL)
        {
            Serial.println("System Running in MANUAL Mode");
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* ==============================
   CREATE TASKS
   ============================== */
void RTOS_CreateTasks(void)
{
    g_lidarQueue = xQueueCreate(1, sizeof(lidar_data_t));
    g_proxiQueue = xQueueCreate(1, sizeof(proxi_data_t));
    g_systemEventGroup = xEventGroupCreate();

    CAN_Driver_Init();
    Proxi_Driver_Init();
    Button_Driver_Init();

    xTaskCreatePinnedToCore(LidarTask, "LidarTask", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(ProxiTask, "ProxiTask", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(ButtonTask, "ButtonTask", 4096, NULL, 4, NULL, 1);

    xTaskCreatePinnedToCore(LidarDebugTask, "LidarDebug", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(ProxiDebugTask, "ProxiDebug", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(ModeMonitorTask, "ModeMonitor", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(
        SprayTask,
        "SprayTask",
        4096,
        NULL,
        6,
        NULL,
        1);
        xEventGroupSetBits(g_systemEventGroup, SYSTEM_BIT_MANUAL);
}