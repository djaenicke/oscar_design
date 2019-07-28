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

#define RAD_PER_REV      (6.2831853f)
#define CLK_PERIOD       (0.00002048f)
#define START            ((uint8_t)0)
#define END              ((uint8_t)1)

#define MAX_MEASUREMENTS 50

#define E_PULSES_PER_REV  ((uint8_t)20)
#define HE_PULSES_PER_REV ((uint8_t)192)

#define E_MIN_PERIOD  513
#define HE_MIN_PERIOD 28

typedef struct {
   uint8_t  meas_type;
   uint8_t  num_meas;
   uint16_t start_cnt;
   uint16_t period_cnt[MAX_MEASUREMENTS];
} Period_T;

static volatile Period_T Sensor_Period[NUM_WHEEL_SENSORS] = {0};

static inline void  Measure_Period(Wheel_Sensor_T pos, uint16_t min_period);
static inline float Period_2_Speed(Wheel_Sensor_T pos, uint8_t pulses_per_rev);

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
   if (NULL != speeds)
   {
      /* Encoder measurement */
      speeds->r = Period_2_Speed(R_E, E_PULSES_PER_REV);
      speeds->l = Period_2_Speed(L_E, E_PULSES_PER_REV);

      /* Hall effect sensor measurement */
      speeds->r_he = Period_2_Speed(R_HE, HE_PULSES_PER_REV);
      speeds->l_he = Period_2_Speed(L_HE, HE_PULSES_PER_REV);
   }
   else
   {
      assert(false);
   }
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
   if (ISR_Flag_Is_Set(L_SPEED_SENSOR_HE))
   {
      Measure_Period(L_HE, HE_MIN_PERIOD);
      Clear_ISR_Flag(L_SPEED_SENSOR_HE);
   }

   if (ISR_Flag_Is_Set(L_SPEED_SENSOR))
   {
      Measure_Period(L_E, E_MIN_PERIOD);
      Clear_ISR_Flag(L_SPEED_SENSOR);
   }
}

void PORTB_IRQHandler(void)
{
   if (ISR_Flag_Is_Set(R_SPEED_SENSOR_HE))
   {
      Measure_Period(R_HE, HE_MIN_PERIOD);
      Clear_ISR_Flag(R_SPEED_SENSOR_HE);
   }

   if (ISR_Flag_Is_Set(R_SPEED_SENSOR))
   {
      Measure_Period(R_E, E_MIN_PERIOD);
      Clear_ISR_Flag(R_SPEED_SENSOR);
   }
}
}

static inline void Measure_Period(Wheel_Sensor_T sensor, uint16_t min_period)
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

      if (period_cnt > min_period)
      {
         Sensor_Period[sensor].period_cnt[Sensor_Period[sensor].num_meas] = ((uint16_t) FTM1->CNT) - Sensor_Period[sensor].start_cnt;
         Sensor_Period[sensor].num_meas++;
      }

      Sensor_Period[sensor].meas_type = START;
   }
}

static inline float Period_2_Speed(Wheel_Sensor_T sensor, uint8_t pulses_per_rev)
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
      temp = RAD_PER_REV/(pulses_per_rev*period*CLK_PERIOD);
      Sensor_Period[sensor].num_meas = 0;
   }

   return temp;
}
