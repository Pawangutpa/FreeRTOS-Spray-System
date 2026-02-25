#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

extern QueueHandle_t qGpsData;
extern QueueHandle_t qMqttPublish;