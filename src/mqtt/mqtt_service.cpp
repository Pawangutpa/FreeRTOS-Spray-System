#include <Arduino.h>
#include "mqtt/mqtt_service.h"
#include "config/mqtt_config.h"

static uint32_t crc32(const char *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFF;
    while (length--)
    {
        uint8_t byte = *data++;
        crc ^= byte;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320 : crc >> 1;
    }
    return ~crc;
}

void MQTT_buildHeartbeat(MqttMessage &msg, const GpsData &gps)
{
    double lat = gps.lat;
double lon = gps.lon;

if (gps.fix == 0)
{
    lat = 0;
    lon = 0;
}
    uint32_t uptime = millis()/1000;
    uint32_t free_heap = ESP.getFreeHeap();
    float cpu_temp = temperatureRead();

    int len = sprintf(msg.payload,
        "[%lu,%d,%d,%lu,%lu,%.6f,%.6f,0,0,0,0,0,0,0,0,0,0,0,%.2f,",
        uptime,
        gps.fix,
        gps.rssi,
        free_heap,
        uptime,
        gps.lat,
        gps.lon,
        cpu_temp
    );

    uint32_t crc = crc32(msg.payload, len);
    sprintf(msg.payload + len, "%lu]", crc);

    sprintf(msg.topic, "/%s/heartbeat", MQTT_DEVICE_ID);

    Serial.println("===== JSON =====");
    Serial.println(msg.payload);
}