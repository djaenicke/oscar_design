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

#define PULSES_PER_REV   (20.0f)
#define RAD_PER_REV      (6.2831853f)
#define CLK_PERIOD       (0.00002048f)

#define ZERO_SPEED_DELAY (2)
#define MAX_MEASUREMENTS (5)
#define MIN_PERIOD_CNTS  (512)

#define START (0)
#define END   (1)

typedef struct {
   uint8_t  num_zero_meas;
   uint8_t  meas_type;
   uint16_t start_cnt;
   uint16_t elapsed_cnt[MAX_MEASUREMENTS];
} Timing_T;

static volatile Timing_T Pulse_Timing[4] = {0};
static volatile uint32_t Pulses[NUM_WHEELS] = {0, 0, 0, 0};

static inline void  Record_Pulse_Info(uint8_t pos);
static inline float Compute_Speed(uint8_t pos, float current_speed);

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

      speeds->rr = Compute_Speed(RR, speeds->rr);
      speeds->rl = Compute_Speed(RL, speeds->rl);
      speeds->fr = Compute_Speed(FR, speeds->fr);
      speeds->fl = Compute_Speed(FL, speeds->fl);

      PORT_ClearPinsInterruptFlags(PORTB, 0xFFFFFFFF);
      EnableIRQ(PORTB_IRQn);
      PORT_ClearPinsInterruptFlags(PORTC, 0xFFFFFFFF);
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
   uint16_t temp_cnt;

   if (START == Pulse_Timing[pos].meas_type)
   {
      Pulse_Timing[pos].start_cnt = (uint16_t)FTM1->CNT;
      Pulse_Timing[pos].meas_type = END;
   }
   else
   {
      /* Sample the period as many times as possible */
      if (Pulses[pos] < MAX_MEASUREMENTS)
      {
         temp_cnt = (uint16_t)FTM1->CNT - Pulse_Timing[pos].start_cnt;

         if (temp_cnt > MIN_PERIOD_CNTS)
         {
            Pulse_Timing[pos].elapsed_cnt[Pulses[pos]] = temp_cnt;
            Pulses[pos]++;
         }

         Pulse_Timing[pos].meas_type = START;
      }
      else
      {
         /* Shouldn't happen */
         assert(false);
      }
   }
}

static inline float Compute_Speed(uint8_t pos, float current_speed)
{
   float temp = 0;

   if (Pulses[pos])
   {
      Pulse_Timing[pos].num_zero_meas = 0;

      /* Compute the average period over all the available measurements */
      for (uint8_t i=0; i<Pulses[pos]; i++)
      {
         temp += RAD_PER_REV/(PULSES_PER_REV*Pulse_Timing[pos].elapsed_cnt[i]*CLK_PERIOD);
      }

      temp /= Pulses[pos];
      Pulses[pos] = 0;
   }
   else if (Pulse_Timing[pos].num_zero_meas < ZERO_SPEED_DELAY)
   {
      Pulse_Timing[pos].num_zero_meas++;
      temp = current_speed;
   }
   else
   {
      temp = 0.0f;
   }

   Pulse_Timing[pos].meas_type = START;
   return temp;
}
