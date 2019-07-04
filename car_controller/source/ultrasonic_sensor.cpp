/*
 * ultrasonic_sensor.cpp
 *
 *  Created on: Jul 4, 2019
 *      Author: Devin
 */

#include "ultrasonic_sensor.h"
#include "io_abstraction.h"
#include "clock_config.h"
#include "fsl_common.h"
#include "fsl_ftm.h"

#define FTM_SOURCE_CLOCK (CLOCK_GetFreq(kCLOCK_BusClk)/4)
#define TRIG_PULSE_TIME (15) /* microseconds */

static volatile Sensor_State_T state = IDLE;

void UltrasonicSensor::Init(void)
{
   ftm_config_t ftm_info;

   FTM_GetDefaultConfig(&ftm_info);

   /* Divide FTM clock by 4 */
   ftm_info.prescale = kFTM_Prescale_Divide_4;

   FTM_Init(FTM2, &ftm_info);
   EnableIRQ(FTM2_IRQn);
}

void UltrasonicSensor::Trigger(void)
{
   FTM_SetTimerPeriod(FTM2, USEC_TO_COUNT(15, FTM_SOURCE_CLOCK));

   Set_GPIO(USS_TRIGGER, HIGH);
   state = TRIG;

   FTM_StartTimer(FTM2, kFTM_SystemClock);

   FTM_EnableInterrupts(FTM2, kFTM_TimeOverflowInterruptEnable);
}

extern "C"
{
void FTM2_IRQHandler(void)
{
   FTM_StopTimer(FTM2);
   FTM2->CNT = 0;

   if (TRIG == state)
   {
      Set_GPIO(USS_TRIGGER, LOW);
      state = ECHO;
   }

   FTM_DisableInterrupts(FTM2, kFTM_TimeOverflowInterruptEnable);

   /* Clear interrupt flag.*/
    FTM_ClearStatusFlags(FTM2, kFTM_TimeOverflowFlag);
    __DSB();
}
}
