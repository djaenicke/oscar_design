/*
 * ultrasonic_sensor.cpp
 *
 *  Created on: Jul 4, 2019
 *      Author: Devin
 */

#include "ultrasonic_sensor.h"
#include "clock_config.h"
#include "assert.h"
#include "fsl_common.h"

extern "C"
{
#include "ftm_isr_router.h"
}

#define FTM_SOURCE_CLOCK (CLOCK_GetFreq(kCLOCK_BusClk)/4)
#define TRIG_PULSE_TIME (15) /* microseconds */

static volatile USS_Working_Info_T * USS_Working_Info[NUM_FTMS];

extern "C"
{
static void USS_FTM_IRQHandler(uint8_t ftm_num);
}

void UltrasonicSensor::Init(FTM_Type *ftm_base_ptr, IO_Map_T trig_pin, IO_Map_T echo_pin)
{
   ftm_config_t ftm_info;
   uint8_t ftm_num = 0;

   assert(ftm_base_ptr);

   switch((uint32_t)ftm_base_ptr)
   {
      case FTM0_BASE:
         EnableIRQ(FTM0_IRQn);
         ftm_num = 0;
         break;
      case FTM1_BASE:
         EnableIRQ(FTM1_IRQn);
         ftm_num = 1;
         break;
      case FTM2_BASE:
         EnableIRQ(FTM2_IRQn);
         ftm_num = 2;
         break;
      case FTM3_BASE:
         EnableIRQ(FTM3_IRQn);
         ftm_num = 3;
         break;
      default:
         /* Invalid ftm_ptr */
         assert(false);
   }

   working_info.ftm_ptr = ftm_base_ptr;
   working_info.trig = trig_pin;
   working_info.echo = echo_pin;

   FTM_GetDefaultConfig(&ftm_info);

   /* Divide FTM clock by 4 */
   ftm_info.prescale = kFTM_Prescale_Divide_4;

   FTM_Init(working_info.ftm_ptr, &ftm_info);

   Reroute_FTM_ISR(ftm_num, &USS_FTM_IRQHandler);
   USS_Working_Info[ftm_num] = &working_info;
}

void UltrasonicSensor::Trigger(void)
{
   FTM_SetTimerPeriod(working_info.ftm_ptr, USEC_TO_COUNT(TRIG_PULSE_TIME, FTM_SOURCE_CLOCK));

   Set_GPIO(working_info.trig, HIGH);
   working_info.state = TRIG;

   FTM_StartTimer(working_info.ftm_ptr, kFTM_SystemClock);

   FTM_EnableInterrupts(working_info.ftm_ptr, kFTM_TimeOverflowInterruptEnable);
}

extern "C"
{
void USS_FTM_IRQHandler(uint8_t ftm_num)
{
   FTM_StopTimer(USS_Working_Info[ftm_num]->ftm_ptr);
   USS_Working_Info[ftm_num]->ftm_ptr->CNT = 0;

   if (TRIG == USS_Working_Info[ftm_num]->state)
   {
      Set_GPIO(USS_Working_Info[ftm_num]->trig, LOW);
      USS_Working_Info[ftm_num]->state = ECHO;
   }

   FTM_DisableInterrupts(USS_Working_Info[ftm_num]->ftm_ptr, kFTM_TimeOverflowInterruptEnable);

   /* Clear interrupt flag.*/
    FTM_ClearStatusFlags(USS_Working_Info[ftm_num]->ftm_ptr, kFTM_TimeOverflowFlag);
    __DSB();
}
}
