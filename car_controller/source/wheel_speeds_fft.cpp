/*
 * wheel_speeds_fft.cpp
 *
 *  Created on: Dec 1, 2019
 *      Author: Devin
 */
#include <stdint.h>

#include "wheel_speeds_fft.h"
#include "fsl_port.h"
#include "io_abstraction.h"
#include "assert.h"
#include "fsl_ftm.h"
#include "interrupt_prios.h"

extern "C"
{
#include "ftm_isr_router.h"
}

#define fs 2500 /* Sample frequency */

static bool Timer_Started = false;
static volatile uint16_t Samples = 0;

extern "C"
{
static void FFT_Sample_FTM_IRQHandler(uint8_t ftm_num);
}

void Init_Wheel_Speeds_FFT(void)
{
   ftm_config_t ftmInfo;

   FTM_GetDefaultConfig(&ftmInfo);

   /* Initialize FTM module */
   FTM_Init(FTM1, &ftmInfo);
   FTM_SetTimerPeriod(FTM1, (1.0/fs)/(1.0f/CLOCK_GetFreq(kCLOCK_BusClk)));
   Reroute_FTM_ISR((uint8_t)1, &FFT_Sample_FTM_IRQHandler);
   EnableIRQ(FTM1_IRQn);
   FTM_EnableInterrupts(FTM1, kFTM_TimeOverflowInterruptEnable);
}

void Get_Wheel_Speeds_FFT(Wheel_Speeds_T * speeds)
{
   assert(speeds);

   if (!Timer_Started)
   {
      FTM_StartTimer(FTM1, kFTM_SystemClock);
      Timer_Started = true;
   }
   else
   {
      FTM_DisableInterrupts(FTM1, kFTM_TimeOverflowInterruptEnable);
      Samples = 0;
      FTM_EnableInterrupts(FTM1, kFTM_TimeOverflowInterruptEnable);
   }
}

extern "C"
{
void FFT_Sample_FTM_IRQHandler(uint8_t ftm_num)
{
   FTM_StopTimer(FTM1);
   FTM1->CNT = 0;
   FTM_StartTimer(FTM1, kFTM_SystemClock);

   Samples++;

   /* Clear interrupt flag.*/
    FTM_ClearStatusFlags(FTM1, kFTM_TimeOverflowFlag);
    __DSB();
}
}
