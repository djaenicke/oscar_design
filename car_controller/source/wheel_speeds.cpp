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

#define PULSES_PER_REV (20.0f)
#define RAD_PER_REV    (6.2831853f)
#define CLK_PERIOD     (0.00002048f)

#define START (0)
#define END   (1)

typedef struct {
   uint8_t  meas_type;
   uint16_t start_cnt;
   uint16_t elapsed_cnt;
} Timing_T;

static volatile Timing_T Pulse_Timing[4] = {0};
static volatile uint32_t Pulses[NUM_WHEELS] = {0, 0, 0, 0};

static inline void Record_Pulse_Info(uint8_t pos);
static inline float Compute_Speed(uint8_t pos);

void Init_Wheel_Speed_Sensors(void)
{
   port_interrupt_t p_int_cfg;
   ftm_config_t ftmInfo;

   FTM_GetDefaultConfig(&ftmInfo);

   ftmInfo.prescale = kFTM_Prescale_Divide_32;

   /* Initialize FTM module */
   FTM_Init(FTM1, &ftmInfo);

   FTM_SetTimerPeriod(FTM1, UINT16_MAX);
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
      DisableIRQ(PORTB_IRQn);
      DisableIRQ(PORTC_IRQn);

      speeds->rr = Compute_Speed(RR);
      speeds->rl = Compute_Speed(RL);
      speeds->fr = Compute_Speed(FR);
      speeds->fl = Compute_Speed(FL);

      EnableIRQ(PORTB_IRQn);
      EnableIRQ(PORTC_IRQn);
   }
   else
   {
      assert(false);
   }
}

extern "C"
{
void PORTC_IRQHandler(void)
{
   /* Determine which wheel speed sensor caused the interrupt */
   if ((Pin_Cfgs[RL_SPEED_SENSOR].pbase->PCR[Pin_Cfgs[RL_SPEED_SENSOR].pin] >> PORT_PCR_ISF_SHIFT) && (uint32_t) 0x01)
   {
      Record_Pulse_Info(RL);
      Pin_Cfgs[RL_SPEED_SENSOR].pbase->PCR[Pin_Cfgs[RL_SPEED_SENSOR].pin] |= PORT_PCR_ISF(1);
   }
   else if ((Pin_Cfgs[FR_SPEED_SENSOR].pbase->PCR[Pin_Cfgs[FR_SPEED_SENSOR].pin] >> PORT_PCR_ISF_SHIFT) && (uint32_t) 0x01)
   {
      Record_Pulse_Info(FR);
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
      Record_Pulse_Info(RR);
      Pin_Cfgs[RR_SPEED_SENSOR].pbase->PCR[Pin_Cfgs[RR_SPEED_SENSOR].pin] |= PORT_PCR_ISF(1);
   }
   else if ((Pin_Cfgs[FL_SPEED_SENSOR].pbase->PCR[Pin_Cfgs[FL_SPEED_SENSOR].pin] >> PORT_PCR_ISF_SHIFT) && (uint32_t) 0x01)
   {
      Record_Pulse_Info(FL);
      Pin_Cfgs[FL_SPEED_SENSOR].pbase->PCR[Pin_Cfgs[FL_SPEED_SENSOR].pin] |= PORT_PCR_ISF(1);
   }
   else
   {
      assert(false);
   }
}
}

static inline void Record_Pulse_Info(uint8_t pos)
{
   Pulses[pos]++;

   if (START == Pulse_Timing[pos].meas_type)
   {
      Pulse_Timing[pos].start_cnt = (uint16_t)FTM1->CNT;
      Pulse_Timing[pos].meas_type = END;
   }
   else
   {
      Pulse_Timing[pos].elapsed_cnt = (uint16_t)FTM1->CNT - Pulse_Timing[pos].start_cnt;
      Pulse_Timing[pos].meas_type = START;
   }
}

static inline float Compute_Speed(uint8_t pos)
{
   float temp;

   if (Pulses[pos])
   {
      temp = RAD_PER_REV/(PULSES_PER_REV*Pulse_Timing[pos].elapsed_cnt*CLK_PERIOD);
      Pulses[pos] = 0;
   }
   else
   {
      temp = 0.0f;
   }

   return temp;
}
