#pragma once
#include "common/telemetry_types.h"

void MQTT_buildHeartbeat(MqttMessage &msg, const GpsData &gps);