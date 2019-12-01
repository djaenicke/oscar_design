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
#include "fsl_sysmpu.h"

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Application */
#include "main.h"
#include "data_logger.h"
#include "inertial_states.h"
#include "io_abstraction.h"
#include "motor_controls.h"
#include "behaviors.h"
#include "servo.h"
#include "wheel_speeds.h"
#include "wheel_speeds_fft.h"
#include "bluetooth_control.h"
#include "battery_monitor.h"
#include "interrupt_prios.h"
#include "logging_streams.h"
#include "object_detection.h"
#include "fft_test.h"
#include "debug_constants.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SD_CARD_INIT_TASK_PRIO     (configMAX_PRIORITIES - 1U)
#define MC_DATA_LOGGING_TASK_PRIO  (configMAX_PRIORITIES - 2U)
#define BEHAVIORS_TASK_PRIO        (configMAX_PRIORITIES - 3U)
#define BLUETOOTH_CMD_TASK_PRIO    (configMAX_PRIORITIES - 4U)

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
   NVIC_SetPriority(BOARD_SDHC_IRQ, SDHC_INT_PRIO);
   SYSMPU_Enable(SYSMPU, false);

#if RUN_FFT_TEST
   Run_FFT_Test();
#endif

   Init_App();
   Set_GPIO(BLUE_LED, LOW);

   /* Create OS Tasks */
   xTaskCreate(SD_Card_Init_Task,    "SD_Card_Init_Task", 1024, NULL, SD_CARD_INIT_TASK_PRIO,     NULL);
   xTaskCreate(Behaviors_Task,       "Behaviors_Task",    2048, NULL, BEHAVIORS_TASK_PRIO,        NULL);
   xTaskCreate(Bluetooth_Cmd_Task,   "Bluetooth_Control", 1024, NULL, BLUETOOTH_CMD_TASK_PRIO,    NULL);
   xTaskCreate(Log_MC_Stream_Task,   "MC_Logging_Task",   1024, NULL, MC_DATA_LOGGING_TASK_PRIO,  NULL);

   vTaskStartScheduler();
}

void Init_App(void)
{
   Init_Battery_Monitor();
   Init_Inertial_Sensors();
#if USE_FFT_WHEEL_SPEEDS
   Init_Wheel_Speeds_FFT();
#else
   Init_Wheel_Speed_Sensors();
#endif
   Init_Motor_Controls();
   Init_Object_Detection();
   Bluetooth_Serial_Open();
   Create_Streams();

   NVIC_SetPriorityGrouping(0U);
}
