#include <Arduino.h>
#include "services/spray_service.h"
#include "rtos/rtos_queues.h"
#include "rtos/rtos_events.h"
#include "config/board_config.h"
#include "config/spray_config.h"
#include "common/data_types.h"

/* ===============================
   Internal State
================================ */

static bool sol_left_state = false;
static bool sol_right_state = false;

/* ===============================
   Helper: Set Solenoid
================================ */

static void SetSolenoidLeft(bool state)
{
    digitalWrite(SOLENOID_LEFT_PIN, state ? HIGH : LOW);
    sol_left_state = state;
}

static void SetSolenoidRight(bool state)
{
    digitalWrite(SOLENOID_RIGHT_PIN, state ? HIGH : LOW);
    sol_right_state = state;
}

/* ===============================
   Spray Task
================================ */

void SprayTask(void *pvParameters)
{
    lidar_data_t lidar_data;

    pinMode(SOLENOID_LEFT_PIN, OUTPUT);
    pinMode(SOLENOID_RIGHT_PIN, OUTPUT);

    SetSolenoidLeft(false);
    SetSolenoidRight(false);

    while (true)
    {
        /* Read current system mode */
        EventBits_t bits =
            xEventGroupGetBits(g_systemEventGroup);

        bool autonomous =
            (bits & SYSTEM_BIT_AUTONOMOUS);

        if (autonomous)
        {
            /* Get latest LiDAR data */
            if (xQueueReceive(g_lidarQueue,
                              &lidar_data,
                              pdMS_TO_TICKS(50)))
            {
                /* LEFT SIDE */
                if (lidar_data.left_is_valid &&
                    lidar_data.left_distance_cm >= SPRAY_MIN_DISTANCE_CM &&
                    lidar_data.left_distance_cm <= SPRAY_MAX_DISTANCE_CM)
                {
                    SetSolenoidLeft(true);
                }
                else
                {
                    SetSolenoidLeft(false);
                }

                /* RIGHT SIDE */
                if (lidar_data.right_is_valid &&
                    lidar_data.right_distance_cm >= SPRAY_MIN_DISTANCE_CM &&
                    lidar_data.right_distance_cm <= SPRAY_MAX_DISTANCE_CM)
                {
                    SetSolenoidRight(true);
                }
                else
                {
                    SetSolenoidRight(false);
                }
            }
        }
        else
        {
            /* Manual mode → force OFF */
            SetSolenoidLeft(false);
            SetSolenoidRight(false);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}