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

#define RAD_PER_REV      (6.2831853f)
#define CLK_PERIOD       (0.00002048f)
#define START            ((uint8_t)0)
#define END              ((uint8_t)1)

#define MAX_MEASUREMENTS 50
#define HE_PULSES_PER_REV ((uint8_t)192)

typedef struct {
   uint8_t  meas_type;
   uint8_t  num_meas;
   uint16_t start_cnt;
   uint16_t period_cnt[MAX_MEASUREMENTS];
} Period_T;

static volatile Period_T Sensor_Period[NUM_WHEELS] = {0};

static inline void  Measure_Period(Wheel_Sensor_T pos);
static inline float Period_2_Speed(Wheel_Sensor_T pos);

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
   assert(speeds);

   /* Hall effect sensor measurement */
   speeds->r = Period_2_Speed(R);
   speeds->l = Period_2_Speed(L);
}

void Zero_Wheel_Speed(Wheel_Sensor_T sensor)
{
   Sensor_Period[sensor].meas_type = START;
   (void) memset((void *)Sensor_Period[sensor].period_cnt, 0, MAX_MEASUREMENTS*sizeof(uint16_t));
}

extern "C"
{
void PORTC_IRQHandler(void)
{
   Measure_Period(L);
   PORT_ClearPinsInterruptFlags(Pin_Cfgs[L_SPEED_SENSOR].pbase, 0xFFFFFFFF);
}

void PORTB_IRQHandler(void)
{
   Measure_Period(R);
   PORT_ClearPinsInterruptFlags(Pin_Cfgs[R_SPEED_SENSOR].pbase, 0xFFFFFFFF);
}
}

static inline void Measure_Period(Wheel_Sensor_T sensor)
{
   uint16_t period_cnt = 0;

   if (START == Sensor_Period[sensor].meas_type)
   {
      Sensor_Period[sensor].start_cnt = (uint16_t) FTM1->CNT;
      Sensor_Period[sensor].meas_type = END;
   }
   else
   {
      period_cnt = ((uint16_t) FTM1->CNT) - Sensor_Period[sensor].start_cnt;
      Sensor_Period[sensor].period_cnt[Sensor_Period[sensor].num_meas] = ((uint16_t) FTM1->CNT) - Sensor_Period[sensor].start_cnt;
      Sensor_Period[sensor].num_meas++;
      Sensor_Period[sensor].meas_type = START;
   }
}

static inline float Period_2_Speed(Wheel_Sensor_T sensor)
{
   uint16_t period = 0;
   float temp = 0.0f;

   for (uint8_t i=0; i<Sensor_Period[sensor].num_meas; i++)
   {
      period += Sensor_Period[sensor].period_cnt[i];
      Sensor_Period[sensor].period_cnt[i] = 0;
   }

   if (period)
   {
      period /= Sensor_Period[sensor].num_meas;
      temp = RAD_PER_REV/(HE_PULSES_PER_REV*period*CLK_PERIOD);
      Sensor_Period[sensor].num_meas = 0;
   }

   return temp;
}
