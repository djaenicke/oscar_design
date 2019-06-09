/*
 * bluetooth_control.cpp
 *
 *  Created on: Jun 9, 2019
 *      Author: Devin
 */

#include "fsl_uart.h"
#include "motor_controls.h"
#include "assert.h"

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

void Process_Bluetooth_Cmd(void)
{
   if (New_Cmd)
   {
      switch (Cmd)
      {
        case 'f': Forward();  break;
        case 'b': Backward(); break;
        case 'l': Left();     break;
        case 'r': Right();    break;
        case 's': Stop();     break;
        case 'a': break;
        default: break;
      }

      New_Cmd = false;
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
