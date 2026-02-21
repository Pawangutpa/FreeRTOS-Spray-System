#include <Arduino.h>
#include "drivers/button_driver.h"
#include "rtos/rtos_events.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "config/board_config.h"


/* ==============================
   DRIVER INIT
   ============================== */
void Button_Driver_Init(void)
{
    pinMode(BUTTON_AUTONOMOUS_PIN, INPUT_PULLUP);
    pinMode(BUTTON_LEFT_PIN, INPUT_PULLUP);
    pinMode(BUTTON_RIGHT_PIN, INPUT_PULLUP);
    pinMode(BUTTON_BOTH_PIN, INPUT_PULLUP);

    Serial.println("Button Driver Started");
}

/* ==============================
   BUTTON TASK
   ============================== */
void ButtonTask(void *pvParameters)
{
    bool lastAutonomousState = false;

    while (true)
    {
        bool autonomousPressed =
            (digitalRead(BUTTON_AUTONOMOUS_PIN) == LOW);

        if (autonomousPressed != lastAutonomousState)
        {
            if (autonomousPressed)
            {
                xEventGroupSetBits(g_systemEventGroup,
                                   SYSTEM_BIT_AUTONOMOUS);

                xEventGroupClearBits(g_systemEventGroup,
                                     SYSTEM_BIT_MANUAL);

                Serial.println("Mode: AUTONOMOUS");
            }
            else
            {
                xEventGroupSetBits(g_systemEventGroup,
                                   SYSTEM_BIT_MANUAL);

                xEventGroupClearBits(g_systemEventGroup,
                                     SYSTEM_BIT_AUTONOMOUS);

                Serial.println("Mode: MANUAL");
            }

            lastAutonomousState = autonomousPressed;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}