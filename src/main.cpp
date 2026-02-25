#include <Arduino.h>
#include "rtos/rtos_queues.h"
#include "drivers/ec200u_driver.h"
#include "mqtt/mqtt_service.h"
static GpsData latestGps = {};
QueueHandle_t qGpsData;

void ecTask(void *pv)
{
    EC200U_init();
    while (1)
    {
        EC200U_process();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void mqttTask(void *pv)
{
    MqttMessage msg;
    GpsData incoming;

    uint32_t lastPublish = 0;

    while (1)
    {
        // Non-blocking queue receive
        if (xQueueReceive(qGpsData, &incoming, 0))
        {
            // Update latest values only
            if (incoming.lat != 0) latestGps.lat = incoming.lat;
            if (incoming.lon != 0) latestGps.lon = incoming.lon;
            if (incoming.fix != 0) latestGps.fix = incoming.fix;
            if (incoming.rssi != 0) latestGps.rssi = incoming.rssi;
        }

        // Publish every 1 second
        if (millis() - lastPublish >= 1000)
        {
            lastPublish = millis();

            if (EC200U_isReady())
            {
                MQTT_buildHeartbeat(msg, latestGps);
                EC200U_publish(msg.topic, msg.payload);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void setup()
{
    Serial.begin(115200);

    qGpsData = xQueueCreate(5, sizeof(GpsData));

    xTaskCreatePinnedToCore(ecTask, "EC200U", 8192, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(mqttTask, "MQTT", 8192, NULL, 5, NULL, 0);
}

void loop() {}