#include <Arduino.h>
#include "drivers/can_driver.h"
#include "rtos/rtos_tasks.h"

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println("System Booting...");

    

    /* Create FreeRTOS tasks */
    RTOS_CreateTasks();
}

void loop()
{
    /*
     * Nothing here.
     * FreeRTOS scheduler handles everything.
     */
}