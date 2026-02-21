#include "drivers/can_driver.h"
#include "rtos/rtos_queues.h"
#include "common/data_types.h"

/* ==============================
   CAN PIN CONFIGURATION
   ============================== */

static const gpio_num_t CAN_TX_PIN = (gpio_num_t)36;
static const gpio_num_t CAN_RX_PIN = (gpio_num_t)37;

/* ==============================
   CAN INITIALIZATION
   ============================== */

void CAN_Driver_Init(void)
{
    twai_general_config_t general_config =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);

    twai_timing_config_t timing_config =
        TWAI_TIMING_CONFIG_500KBITS();

    twai_filter_config_t filter_config =
        TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&general_config, &timing_config, &filter_config) == ESP_OK)
    {
        Serial.println("CAN Driver Installed");
    }

    if (twai_start() == ESP_OK)
    {
        Serial.println("CAN Driver Started");
    }
}

/* ==============================
   LIDAR TASK (FreeRTOS)
   ============================== */

void LidarTask(void *pvParameters)
{
    twai_message_t rx_message;
    lidar_data_t lidar_data = {0};

    while (true)
    {
        /*
         * Wait up to 100ms for CAN frame
         */
        if (twai_receive(&rx_message, pdMS_TO_TICKS(100)) == ESP_OK)
        {
            /*
             * Frame ID 0x101 → Left LiDAR
             * Frame ID 0x102 → Right LiDAR
             */

            if (rx_message.identifier == 0x101)
            {
                lidar_data.left_distance_cm =
                    rx_message.data[0] | (rx_message.data[1] << 8);

                lidar_data.left_signal_strength =
                    rx_message.data[2] | (rx_message.data[3] << 8);
            }
            else if (rx_message.identifier == 0x102)
            {
                lidar_data.right_distance_cm =
                    rx_message.data[0] | (rx_message.data[1] << 8);

                lidar_data.right_signal_strength =
                    rx_message.data[2] | (rx_message.data[3] << 8);
            }

            /*
             * Send updated structure to queue
             * Non-blocking send
             */
            xQueueSend(g_lidarQueue, &lidar_data, 0);
        }

        /*
         * Yield CPU for 10ms
         */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}