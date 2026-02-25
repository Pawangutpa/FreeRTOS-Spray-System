#pragma once
#include "common/telemetry_types.h"

void EC200U_init();
void EC200U_process();
void EC200U_publish(const char *topic, const char *payload);
bool EC200U_isReady();