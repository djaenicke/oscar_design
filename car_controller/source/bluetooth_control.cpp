/*
 * bluetooth_control.cpp
 *
 *  Created on: Jun 9, 2019
 *      Author: Devin
 */

#include "FreeRTOS.h"
#include "task.h"

#include "data_logger.h"
#include "fsl_uart.h"
#include "motor_controls.h"
#include "assert.h"
#include "interrupt_prios.h"
#include "object_detection.h"

#define RX_BUFFER_SIZE 10

static uart_config_t UART_Cfg;
static volatile bool New_Cmd = false;
static volatile uint8_t Cmd = 0;

void Bluetooth_Serial_Open(void)
{
    status_t status;

    UART_GetDefaultConfig(&UART_Cfg);
    UART_Cfg.baudRate_Bps = 9600;
    UART_Cfg.enableTx = true;
    UART_Cfg.enableRx = true;

    status = UART_Init(UART4, &UART_Cfg, CLOCK_GetFreq(kCLOCK_BusClk));

    NVIC_SetPriority(UART4_RX_TX_IRQn, UART4_INT_PRIO);

    UART_EnableInterrupts(UART4, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable);
    EnableIRQ(UART4_RX_TX_IRQn);

    if (kStatus_Success != status)
    {
       assert(false);
    }
}

void Bluetooth_Serial_Close(void)
{
   UART_Deinit(UART4);
}

void Bluetooth_Cmd_Task(void *pvParameters)
{
   while(1)
   {
      if (New_Cmd)
      {
         switch (Cmd)
         {
           case 'f': Update_Wheel_Speed_Setpoints( 20.0f,  20.0f); break;
           case 'b': Update_Wheel_Speed_Setpoints(-20.0f, -20.0f); break;
           case 'l': Update_Wheel_Speed_Setpoints(-8.0f,    8.0f); break;
           case 'r': Update_Wheel_Speed_Setpoints( 8.0f,   -8.0f); break;
           case 's': Stop(); break;
           case 'a': break;
           case 'o': Open_Log_File();  break;
           case 'c': Close_Log_File(); break;
           case 't': Toggle_Obj_Det_Enable(); break;
         }

         New_Cmd = false;
      }

      vTaskDelay(pdMS_TO_TICKS(10));
   }
}

extern "C"
{
void UART4_RX_TX_IRQHandler(void)
{
   /* If new data arrived. */
   if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(UART4))
   {
      Cmd = UART_ReadByte(UART4);
      New_Cmd = true;
   }
   __DSB();
}
}
