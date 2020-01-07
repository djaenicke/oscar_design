/*
 * delay.cpp
 *
 *  Created on: Dec 31, 2019
 *      Author: Devin
 */

#include "delay.h"

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

extern void Delay_ms(uint16_t delay_t)
{
   TickType_t start_tick = xTaskGetTickCount();
   TickType_t delay_ticks;

   delay_ticks = pdMS_TO_TICKS(delay_t);

   while((xTaskGetTickCount() - start_tick) < delay_ticks) {};
}

