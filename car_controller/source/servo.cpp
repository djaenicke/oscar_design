/*
 * servo.cpp
 *
 *  Created on: Jun 8, 2019
 *      Author: Devin
 */

#include <math.h>

#include "servo.h"
#include "io_abstraction.h"
#include "fsl_ftm.h"
#include "clock_config.h"
#include "assert.h"
#include "fsl_common.h"

extern "C"
{
#include "ftm_isr_router.h"
}

#define FTM_SOURCE_CLOCK (CLOCK_GetFreq(kCLOCK_BusClk)/32)
#define SERVO_PWM_FREQ (50.0f) /* Hz */
#define SEC_2_US (1000000)
#define SERVO_PERIOD (uint16_t)((1/SERVO_PWM_FREQ)*SEC_2_US)

#define MIN_PULSE_TIME    (400.0f) /* us */
#define MAX_PULSE_WIDTH  (2000.0f) /* us */

#define ANGLE_2_PULSE_WIDTH_TIME(angle) roundf((((angle)*MAX_PULSE_WIDTH/MAX_ANGLE_DEG)+MIN_PULSE_TIME))

static volatile Software_PWM_T *PWM[NUM_FTMS];

extern "C"
{
static void Servo_FTM_IRQHandler(uint8_t ftm_num);
}

void Servo::Init(float offset, FTM_Type *ftm_base_ptr)
{
   /* Configure the PWM output */
   ftm_config_t ftmInfo;
   uint8_t ftm_num = 0;

   assert(ftm_base_ptr);

   position_offset = offset;
   min_angle += position_offset;
   max_angle -= position_offset;

   /* Initialize the software based PWM parameters */
   pwm.ftm_ptr  = ftm_base_ptr;
   pwm.period   = SERVO_PERIOD;
   pwm.on_time  = ANGLE_2_PULSE_WIDTH_TIME(cur_angle + position_offset);
   pwm.off_time = (pwm.period - pwm.on_time);
   pwm.dc_state = DC_ON;

   FTM_GetDefaultConfig(&ftmInfo);

   /* Divide FTM clock by 32 */
   ftmInfo.prescale = kFTM_Prescale_Divide_32;

   FTM_Init(ftm_base_ptr, &ftmInfo);

   FTM_SetTimerPeriod(pwm.ftm_ptr, USEC_TO_COUNT(pwm.on_time, FTM_SOURCE_CLOCK));

   FTM_EnableInterrupts(pwm.ftm_ptr, kFTM_TimeOverflowInterruptEnable);

   switch((uint32_t)pwm.ftm_ptr)
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

   Reroute_FTM_ISR(ftm_num, &Servo_FTM_IRQHandler);
   PWM[ftm_num] = &pwm;

   Set_GPIO(SERVO, HIGH);

   FTM_StartTimer(pwm.ftm_ptr, kFTM_SystemClock);

   init_complete = true;
}

float Servo::Get_Angle(void)
{
   return(cur_angle);
}

float Servo::Get_Max_Angle(void)
{
   return(max_angle);
}

float Servo::Get_Min_Angle(void)
{
   return(min_angle);
}

void Servo::Set_Angle(float angle)
{
   if (!init_complete)
   {
      /* Init method must be called first */
      assert(false);
   }

   /* Saturate the angle to be within [min_angle, max_angle] */
   cur_angle = angle > max_angle ? max_angle : angle < min_angle ? min_angle : angle;

   FTM_DisableInterrupts(pwm.ftm_ptr, kFTM_TimeOverflowInterruptEnable);

   pwm.on_time = ANGLE_2_PULSE_WIDTH_TIME(cur_angle + position_offset);
   pwm.off_time = (pwm.period - pwm.on_time);

   FTM_EnableInterrupts(pwm.ftm_ptr, kFTM_TimeOverflowInterruptEnable);
}

void Servo::Set_Max_Angle(float angle)
{
   max_angle = angle < max_angle ? angle : max_angle;
}

void Servo::Set_Min_Angle(float angle)
{
   min_angle = angle > min_angle ? angle : min_angle;
}

extern "C"
{
void Servo_FTM_IRQHandler(uint8_t ftm_num)
{
   FTM_StopTimer(PWM[ftm_num]->ftm_ptr);

   if (DC_ON == PWM[ftm_num]->dc_state)
   {
      Set_GPIO(SERVO, LOW);
      FTM_SetTimerPeriod(PWM[ftm_num]->ftm_ptr, USEC_TO_COUNT(PWM[ftm_num]->off_time, FTM_SOURCE_CLOCK));
      PWM[ftm_num]->dc_state = DC_OFF;
   }
   else
   {
      Set_GPIO(SERVO, HIGH);
      FTM_SetTimerPeriod(PWM[ftm_num]->ftm_ptr, USEC_TO_COUNT(PWM[ftm_num]->on_time, FTM_SOURCE_CLOCK));
      PWM[ftm_num]->dc_state = DC_ON;
   }

   FTM_StartTimer(PWM[ftm_num]->ftm_ptr, kFTM_SystemClock);

   /* Clear interrupt flag.*/
    FTM_ClearStatusFlags(PWM[ftm_num]->ftm_ptr, kFTM_TimeOverflowFlag);
    __DSB();
}
}

