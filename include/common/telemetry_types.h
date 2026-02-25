#pragma once
#include <Arduino.h>

typedef struct {
    double lat;
    double lon;
    int fix;
    int rssi;
} GpsData;

typedef struct {
    char topic[128];
    char payload[600];
} MqttMessage;