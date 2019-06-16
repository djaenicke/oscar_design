/*
 * Copyright 2016-2018 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    car_controller.cpp
 * @brief   Application entry point.
 */
#include <stdio.h>

/* SDK includes */
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Application */
#include "car_controller.h"
#include "io_abstraction.h"
#include "motor_controls.h"
#include "servo.h"
#include "wheel_speeds.h"
#include "bluetooth_control.h"
#include "battery_monitor.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
typedef struct Task_Cfg_Tag
{
    TaskFunction_t func;
    const char name[configMAX_TASK_NAME_LEN];
    const configSTACK_DEPTH_TYPE stack_size;
    UBaseType_t priority;
    TaskHandle_t handle;
} Task_Cfg_T;

static void Init_App(void);

/*******************************************************************************
* Variables
******************************************************************************/


/*******************************************************************************
 * Function Definitions
 ******************************************************************************/
int main(void)
{
   /* Init board hardware. */
   BOARD_InitBootPins();
   BOARD_InitBootClocks();
   BOARD_InitBootPeripherals();

   /* Init FSL debug console. */
   BOARD_InitDebugConsole();

   Init_App();
   Set_GPIO(BLUE_LED, LOW);

   /* Create OS Tasks */
   xTaskCreate(Motor_Controls_Task, "Motor_Controls",    1000, NULL, 10, NULL);
   xTaskCreate(Bluetooth_Cmd_Task,  "Bluetooth_Control", 1000, NULL, 11, NULL);

   vTaskStartScheduler();
}

void Init_App(void)
{
   Init_Battery_Monitor();
   Init_Wheel_Speed_Sensors();
   Init_Motor_Controls();

   Bluetooth_Serial_Open();
}
