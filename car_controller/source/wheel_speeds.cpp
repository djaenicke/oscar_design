/*
 * wheel_speeds.cpp
 *
 *  Created on: Jun 14, 2019
 *      Author: Devin
 */

#include <string.h>
#include "wheel_speeds.h"
#include "fsl_port.h"
#include "io_abstraction.h"
#include "assert.h"
#include "fsl_ftm.h"

#define FTM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_McgFixedFreqClk)/32
#define PULSES_PER_REV  (20.0f)
#define RAD_PER_REV     (2*3.1459f)
#define SAMPLE_TIME     (500) /* ms */
#define MS_PER_SEC      (1000.0f)

static volatile uint32_t Pulses[NUM_WHEELS] = {0, 0, 0, 0};
static volatile Wheel_Speeds_T Wheel_Speeds = {0.0, 0.0, 0.0, 0.0};

void Init_Wheel_Speed_Sensors(void)
{
   port_interrupt_t p_int_cfg;
   ftm_config_t ftmInfo;

   FTM_GetDefaultConfig(&ftmInfo);

   ftmInfo.prescale = kFTM_Prescale_Divide_32;

   /* Initialize FTM module */
   FTM_Init(FTM1, &ftmInfo);

   FTM_SetTimerPeriod(FTM1, MSEC_TO_COUNT(SAMPLE_TIME, FTM_SOURCE_CLOCK));

   FTM_EnableInterrupts(FTM1, kFTM_TimeOverflowInterruptEnable);

   EnableIRQ(FTM1_IRQn);

   FTM_StartTimer(FTM1, kFTM_FixedClock);

   /* Enable interrupts to capture the optical encoder pulses */
   p_int_cfg = kPORT_InterruptRisingEdge;
   PORT_SetPinInterruptConfig(Pin_Cfgs[RR_SPEED_SENSOR].pbase, Pin_Cfgs[RR_SPEED_SENSOR].pin, p_int_cfg);
   PORT_SetPinInterruptConfig(Pin_Cfgs[RL_SPEED_SENSOR].pbase, Pin_Cfgs[RL_SPEED_SENSOR].pin, p_int_cfg);
   PORT_SetPinInterruptConfig(Pin_Cfgs[FR_SPEED_SENSOR].pbase, Pin_Cfgs[FR_SPEED_SENSOR].pin, p_int_cfg);
   PORT_SetPinInterruptConfig(Pin_Cfgs[FL_SPEED_SENSOR].pbase, Pin_Cfgs[FL_SPEED_SENSOR].pin, p_int_cfg);

   NVIC_SetPriority(PORTB_IRQn, 2);
   NVIC_SetPriority(PORTC_IRQn, 2);

   PORT_ClearPinsInterruptFlags(PORTB, 0xFFFFFFFF);
   EnableIRQ(PORTB_IRQn);

   PORT_ClearPinsInterruptFlags(PORTC, 0xFFFFFFFF);
   EnableIRQ(PORTC_IRQn);
}

void Get_Wheel_Speeds(Wheel_Speeds_T * speeds)
{
   if (NULL != speeds)
   {
      FTM_DisableInterrupts(FTM1, kFTM_TimeOverflowInterruptEnable);
      (void) memcpy(speeds, (const void *)&Wheel_Speeds, sizeof(Wheel_Speeds_T));
      FTM_EnableInterrupts(FTM1, kFTM_TimeOverflowInterruptEnable);
   }
   else
   {
      assert(false);
   }
}

extern "C"
{
void FTM1_IRQHandler(void)
{
   DisableIRQ(PORTB_IRQn);
   DisableIRQ(PORTC_IRQn);

   Wheel_Speeds.rr = ((Pulses[RR]/PULSES_PER_REV)*RAD_PER_REV)/(SAMPLE_TIME/MS_PER_SEC);
   Pulses[RR] = 0.0f;

   Wheel_Speeds.rl = ((Pulses[RL]/PULSES_PER_REV)*RAD_PER_REV)/(SAMPLE_TIME/MS_PER_SEC);
   Pulses[RL] = 0.0f;

   Wheel_Speeds.fr = ((Pulses[FR]/PULSES_PER_REV)*RAD_PER_REV)/(SAMPLE_TIME/MS_PER_SEC);
   Pulses[FR] = 0.0f;

   Wheel_Speeds.fl = ((Pulses[FL]/PULSES_PER_REV)*RAD_PER_REV)/(SAMPLE_TIME/MS_PER_SEC);
   Pulses[FL] = 0.0f;

   FTM1->CNT = (uint32_t) 0;

   /* Clear interrupt flag.*/
    FTM_ClearStatusFlags(FTM1, kFTM_TimeOverflowFlag);
    __DSB();

   EnableIRQ(PORTB_IRQn);
   EnableIRQ(PORTC_IRQn);
}

void PORTC_IRQHandler(void)
{
   /* Determine which wheel speed sensor caused the interrupt */
   if ((Pin_Cfgs[RL_SPEED_SENSOR].pbase->PCR[Pin_Cfgs[RL_SPEED_SENSOR].pin] >> PORT_PCR_ISF_SHIFT) && (uint32_t) 0x01)
   {
      Pulses[RL]++;
      Pin_Cfgs[RL_SPEED_SENSOR].pbase->PCR[Pin_Cfgs[RL_SPEED_SENSOR].pin] |= PORT_PCR_ISF(1);
   }
   else if ((Pin_Cfgs[FR_SPEED_SENSOR].pbase->PCR[Pin_Cfgs[FR_SPEED_SENSOR].pin] >> PORT_PCR_ISF_SHIFT) && (uint32_t) 0x01)
   {
      Pulses[FR]++;
      Pin_Cfgs[FR_SPEED_SENSOR].pbase->PCR[Pin_Cfgs[FR_SPEED_SENSOR].pin] |= PORT_PCR_ISF(1);
   }
   else
   {
      assert(false);
   }
}

void PORTB_IRQHandler(void)
{
   /* Determine which wheel speed sensor caused the interrupt */
   if ((Pin_Cfgs[RR_SPEED_SENSOR].pbase->PCR[Pin_Cfgs[RR_SPEED_SENSOR].pin] >> PORT_PCR_ISF_SHIFT) && (uint32_t) 0x01)
   {
      Pulses[RR]++;
      Pin_Cfgs[RR_SPEED_SENSOR].pbase->PCR[Pin_Cfgs[RR_SPEED_SENSOR].pin] |= PORT_PCR_ISF(1);
   }
   else if ((Pin_Cfgs[FL_SPEED_SENSOR].pbase->PCR[Pin_Cfgs[FL_SPEED_SENSOR].pin] >> PORT_PCR_ISF_SHIFT) && (uint32_t) 0x01)
   {
      Pulses[FL]++;
      Pin_Cfgs[FL_SPEED_SENSOR].pbase->PCR[Pin_Cfgs[FL_SPEED_SENSOR].pin] |= PORT_PCR_ISF(1);
   }
   else
   {
      assert(false);
   }
}
}
