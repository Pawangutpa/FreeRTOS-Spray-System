#include <Arduino.h>
#include "drivers/can_driver.h"
#include "config/board_config.h"
#include "rtos/rtos_queues.h"
#include "common/data_types.h"
#include "driver/twai.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "config/spray_config.h"

/* ==========================================================
   CAN DRIVER INITIALIZATION
   ========================================================== */

void CAN_Driver_Init(void)
{
    /* ---- General Config ---- */
    twai_general_config_t general_config =
        TWAI_GENERAL_CONFIG_DEFAULT(
            (gpio_num_t)CAN_TX_PIN,
            (gpio_num_t)CAN_RX_PIN,
            TWAI_MODE_NORMAL);

    /* ---- Timing Config (500 kbps) ---- */
    twai_timing_config_t timing_config =
        TWAI_TIMING_CONFIG_500KBITS();

    /* ---- Accept All Frames ---- */
    twai_filter_config_t filter_config =
        TWAI_FILTER_CONFIG_ACCEPT_ALL();

    /* ---- Install Driver ---- */
    if (twai_driver_install(
        &general_config,
        &timing_config,
        &filter_config) == ESP_OK)
    {
        Serial.println("CAN Driver Installed");
    }
    else
    {
        Serial.println("CAN Driver Install Failed");
        return;
    }

    /* ---- Start Driver ---- */
    if (twai_start() == ESP_OK)
    {
        Serial.println("CAN Driver Started");
    }
    else
    {
        Serial.println("CAN Start Failed");
    }
}

/* ==========================================================
   LIDAR TASK
   ========================================================== */

void LidarTask(void* pvParameters)
{
    twai_message_t rx_msg;
    lidar_data_t lidar_data;

    /* Initialize structure to zero */
    memset(&lidar_data, 0, sizeof(lidar_data));

    Serial.println("LidarTask Started");

    while (true)
    {
        /*
         * Wait for CAN frame (timeout 200ms)
         */
        if (twai_receive(&rx_msg, pdMS_TO_TICKS(200)) == ESP_OK)
        {
            if (rx_msg.data_length_code >= 4)
            {
                uint16_t distance =
                    rx_msg.data[0] | (rx_msg.data[1] << 8);

                uint16_t strength =
                    rx_msg.data[2] | (rx_msg.data[3] << 8);

                /* -----------------------------
                   LEFT SENSOR (ID 0x101)
                ----------------------------- */
                if (rx_msg.identifier == 0x101)
                {
                    lidar_data.left_raw_distance_cm = distance;
                    lidar_data.left_raw_strength = strength;

                    /* Basic validity filtering */
                    if (strength > SPRAY_MIN_STRENGTH &&
                        distance > SPRAY_MIN_DISTANCE_CM &&
                        distance < SPRAY_MAX_DISTANCE_CM)
                    {
                        lidar_data.left_distance_cm = distance;
                        lidar_data.left_is_valid = true;
                    }
                    else
                    {
                        lidar_data.left_is_valid = false;
                    }
                }

                /* -----------------------------
                   RIGHT SENSOR (ID 0x102)
                ----------------------------- */
                else if (rx_msg.identifier == 0x102)
                {
                    lidar_data.right_raw_distance_cm = distance;
                    lidar_data.right_raw_strength = strength;

                    if (strength > SPRAY_MIN_STRENGTH &&
                        distance > SPRAY_MIN_DISTANCE_CM &&
                        distance < SPRAY_MAX_DISTANCE_CM)
                    {
                        lidar_data.right_distance_cm = distance;
                        lidar_data.right_is_valid = true;
                    }
                    else
                    {
                        lidar_data.right_is_valid = false;
                    }
                }

                /*
                 * Send latest data to queue
                 * Queue length should be 1
                 */
                if (g_lidarQueue != NULL)
                {
                    xQueueOverwrite(g_lidarQueue, &lidar_data);
                }
            }
        }

        /*
         * Small delay to prevent CPU starvation
         */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}