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

#define HE_PULSES_PER_REV (192)

typedef struct {
   uint8_t  meas_type;
   uint8_t  num_meas;
   uint16_t start_cnt;
   uint16_t period_cnt[MAX_MEASUREMENTS];
} Period_T;

static volatile Period_T Encoder_Period[NUM_WHEELS] = {0};
static volatile uint16_t HE_Sensor_Pulses[NUM_WHEELS] = {0, 0};

static inline void  Measure_Period(Wheel_T pos);
static inline float Period_2_Speed(Wheel_T pos);
static inline float Pulses_2_Speed(Wheel_T pos, float dt);

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
   PORT_SetPinInterruptConfig(Pin_Cfgs[R_SPEED_SENSOR_HE].pbase, Pin_Cfgs[R_SPEED_SENSOR_HE].pin, p_int_cfg);
   PORT_SetPinInterruptConfig(Pin_Cfgs[L_SPEED_SENSOR_HE].pbase, Pin_Cfgs[L_SPEED_SENSOR_HE].pin, p_int_cfg);

   PORT_SetPinInterruptConfig(Pin_Cfgs[R_SPEED_SENSOR].pbase, Pin_Cfgs[R_SPEED_SENSOR].pin, p_int_cfg);
   PORT_SetPinInterruptConfig(Pin_Cfgs[L_SPEED_SENSOR].pbase, Pin_Cfgs[L_SPEED_SENSOR].pin, p_int_cfg);

   NVIC_SetPriority(PORTB_IRQn, ENCODER_INT_PRIO);
   NVIC_SetPriority(PORTC_IRQn, ENCODER_INT_PRIO);

   PORT_ClearPinsInterruptFlags(PORTB, 0xFFFFFFFF);
   EnableIRQ(PORTB_IRQn);

   PORT_ClearPinsInterruptFlags(PORTC, 0xFFFFFFFF);
   EnableIRQ(PORTC_IRQn);
}

void Get_Wheel_Speeds(Wheel_Speeds_T * speeds)
{
   static uint16_t last_cnt = 0;
   uint16_t cnt, delta_cnt;
   float dt;

   cnt = (uint16_t) FTM1->CNT;

   if (last_cnt)
   {
      if (cnt > last_cnt)
      {
         /* No rollover */
         delta_cnt = cnt - last_cnt;
      }
      else
      {
         /* Rollover */
         delta_cnt = (UINT16_MAX - last_cnt) + cnt;
      }

      dt = delta_cnt * CLK_PERIOD;
   }
   else
   {
      /* First time being called assume 50ms */
      dt = 0.05;
   }

   last_cnt = cnt;

   if (NULL != speeds)
   {
      speeds->r = Period_2_Speed(R);
      speeds->l = Period_2_Speed(L);

      speeds->r_he = Pulses_2_Speed(R, dt);
      speeds->l_he = Pulses_2_Speed(L, dt);
   }
   else
   {
      assert(false);
   }
}

void Zero_Wheel_Speed(Wheel_T pos)
{
   Encoder_Period[pos].meas_type = START;

   for (uint8_t i=0; i<(uint8_t)MAX_MEASUREMENTS; i++)
   {
      Encoder_Period[pos].period_cnt[i] = (uint16_t) 0;
   }

   HE_Sensor_Pulses[pos] = 0;
}

extern "C"
{
void PORTC_IRQHandler(void)
{
   if (ISR_Flag_Is_Set(L_SPEED_SENSOR_HE))
   {
      HE_Sensor_Pulses[L]++;
      Clear_ISR_Flag(L_SPEED_SENSOR_HE);
   }

   if (ISR_Flag_Is_Set(L_SPEED_SENSOR))
   {
      Measure_Period(L);
      Clear_ISR_Flag(L_SPEED_SENSOR);
   }
}

void PORTB_IRQHandler(void)
{
   if (ISR_Flag_Is_Set(R_SPEED_SENSOR_HE))
   {
      HE_Sensor_Pulses[R]++;
      Clear_ISR_Flag(R_SPEED_SENSOR_HE);
   }

   if (ISR_Flag_Is_Set(R_SPEED_SENSOR))
   {
      Measure_Period(R);
      Clear_ISR_Flag(R_SPEED_SENSOR);
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

static inline float Pulses_2_Speed(Wheel_T pos, float dt)
{
   float temp = 0.0f;

   if (HE_Sensor_Pulses[pos])
   {
      temp = (((((float)HE_Sensor_Pulses[pos]/HE_PULSES_PER_REV)*RAD_PER_REV)/dt));
   }

   HE_Sensor_Pulses[pos] = 0;

   return temp;
}

