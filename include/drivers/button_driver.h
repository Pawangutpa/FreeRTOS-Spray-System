#ifndef BUTTON_DRIVER_H
#define BUTTON_DRIVER_H

#include <Arduino.h>

void Button_Driver_Init(void);
void ButtonTask(void *pvParameters);

#endif