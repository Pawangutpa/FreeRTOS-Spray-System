#ifndef RTOS_EVENTS_H
#define RTOS_EVENTS_H

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

/* Event Group Handle */
extern EventGroupHandle_t g_systemEventGroup;

/* Event Bits */
#define SYSTEM_BIT_AUTONOMOUS   (1 << 0)
#define SYSTEM_BIT_MANUAL       (1 << 1)

#endif