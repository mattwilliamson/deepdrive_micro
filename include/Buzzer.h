#ifndef BUZZER_H
#define BUZZER_H

#include <Arduino.h>

// #include <FreeRTOS.h>
// #include <task.h>

#include <anyrtttl.h>
#include <binrtttl.h>
#include <pitches.h>

#include "config.h"

void setupBuzzer();
void playBuzzer();
// void vTaskBuzzer(void *pvParameters);

#endif // BUZZER_H