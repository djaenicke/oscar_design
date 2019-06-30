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
#include "interrupt_prios.h"

#define ISR_Flag_Is_Set(pos) ((Pin_Cfgs[pos].pbase->PCR[Pin_Cfgs[pos].pin] >> PORT_PCR_ISF_SHIFT) && (uint32_t) 0x01)
#define Clear_ISR_Flag(pos)  Pin_Cfgs[pos].pbase->PCR[Pin_Cfgs[pos].pin] |= PORT_PCR_ISF(1)

#define PULSES_PER_REV (20.0f)
#define RAD_PER_REV    (6.2831853f)
#define CLK_PERIOD     (0.00002048f)
#define START          ((uint8_t)0)
#define END            ((uint8_t)1)

#define MAX_MEASUREMENTS 5
#define MIN_PERIOD 512

typedef struct {
   uint8_t  meas_type;
   uint8_t  num_meas;
   uint16_t start_cnt;
   uint16_t period_cnt[MAX_MEASUREMENTS];
} Period_T;

static volatile Period_T Encoder_Period[NUM_WHEELS] = {0};

static inline void  Measure_Period(Wheel_T pos);
static inline float Period_2_Speed(Wheel_T pos);

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

   NVIC_SetPriority(PORTB_IRQn, ENCODER_INT_PRIO);
   NVIC_SetPriority(PORTC_IRQn, ENCODER_INT_PRIO);

   PORT_ClearPinsInterruptFlags(PORTB, 0xFFFFFFFF);
   EnableIRQ(PORTB_IRQn);

   PORT_ClearPinsInterruptFlags(PORTC, 0xFFFFFFFF);
   EnableIRQ(PORTC_IRQn);
}

void Get_Wheel_Speeds(Wheel_Speeds_T * speeds)
{
   if (NULL != speeds)
   {
      speeds->rr = Period_2_Speed(RR);
      speeds->rl = Period_2_Speed(RL);
      speeds->fr = Period_2_Speed(FR);
      speeds->fl = Period_2_Speed(FL);
   }
   else
   {
      assert(false);
   }
}

void Zero_Wheel_Speeds(void)
{
   for (uint8_t i=0; i<(uint8_t)NUM_WHEELS; i++)
   {
      for (uint8_t j=0; j<(uint8_t)MAX_MEASUREMENTS; j++)
      {
         Encoder_Period[i].period_cnt[j] = (uint16_t) 0;
      }
      Encoder_Period[i].meas_type = START;
   }
}

void Zero_Left_Wheel_Speeds(void)
{
   Encoder_Period[RL].meas_type = START;
   Encoder_Period[FL].meas_type = START;

   for (uint8_t i=0; i<(uint8_t)MAX_MEASUREMENTS; i++)
   {
      Encoder_Period[RL].period_cnt[i] = (uint16_t) 0;
      Encoder_Period[FL].period_cnt[i] = (uint16_t) 0;
   }
}

void Zero_Right_Wheel_Speeds(void)
{
   Encoder_Period[RR].meas_type = START;
   Encoder_Period[FR].meas_type = START;

   for (uint8_t i=0; i<(uint8_t)MAX_MEASUREMENTS; i++)
   {
      Encoder_Period[RR].period_cnt[i] = (uint16_t) 0;
      Encoder_Period[FR].period_cnt[i] = (uint16_t) 0;
   }
}

extern "C"
{
void PORTC_IRQHandler(void)
{
   /* Determine which wheel speed sensor(s) caused the interrupt */
   if (ISR_Flag_Is_Set(RL_SPEED_SENSOR))
   {
      Measure_Period(RL);
      Clear_ISR_Flag(RL_SPEED_SENSOR);
   }
   if (ISR_Flag_Is_Set(FR_SPEED_SENSOR))
   {
      Measure_Period(FR);
      Clear_ISR_Flag(FR_SPEED_SENSOR);
   }
}

void PORTB_IRQHandler(void)
{
   /* Determine which wheel speed sensor(s) caused the interrupt */
   if (ISR_Flag_Is_Set(RR_SPEED_SENSOR))
   {
      Measure_Period(RR);
      Clear_ISR_Flag(RR_SPEED_SENSOR);
   }
   if (ISR_Flag_Is_Set(FL_SPEED_SENSOR))
   {
      Measure_Period(FL);
      Clear_ISR_Flag(FL_SPEED_SENSOR);
   }
}
}

static inline void Measure_Period(Wheel_T pos)
{
   uint16_t period_cnt = 0;

   if (START == Encoder_Period[pos].meas_type)
   {
      Encoder_Period[pos].start_cnt = (uint16_t) FTM1->CNT;
      Encoder_Period[pos].meas_type = END;
   }
   else
   {
      period_cnt = ((uint16_t) FTM1->CNT) - Encoder_Period[pos].start_cnt;

      if (period_cnt > MIN_PERIOD)
      {
         Encoder_Period[pos].period_cnt[Encoder_Period[pos].num_meas] = ((uint16_t) FTM1->CNT) - Encoder_Period[pos].start_cnt;
         Encoder_Period[pos].num_meas++;
      }

      Encoder_Period[pos].meas_type = START;
   }
}

static inline float Period_2_Speed(Wheel_T pos)
{
   uint16_t period = 0;
   float temp = 0.0f;

   for (uint8_t i=0; i<Encoder_Period[pos].num_meas; i++)
   {
      period += Encoder_Period[pos].period_cnt[i];
      Encoder_Period[pos].period_cnt[i] = 0;
   }

   if (period)
   {
      period /= Encoder_Period[pos].num_meas;
      temp = RAD_PER_REV/(PULSES_PER_REV*period*CLK_PERIOD);
      Encoder_Period[pos].num_meas = 0;
   }

   return temp;
}

