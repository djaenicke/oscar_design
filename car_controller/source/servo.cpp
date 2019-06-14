/*
 * servo.cpp
 *
 *  Created on: Jun 8, 2019
 *      Author: Devin
 */

#include "servo.h"
#include "io_abstraction.h"
#include "fsl_ftm.h"
#include "clock_config.h"
#include "assert.h"
#include "fsl_common.h"

typedef enum {
   DC_OFF,
   DC_ON
} PWM_DC_State_T;

typedef struct {
   PWM_DC_State_T dc_state;
   uint16_t on_time;
   uint16_t off_time;
   uint16_t period;
} Software_PWM_T;

#define FTM_SOURCE_CLOCK (CLOCK_GetFreq(kCLOCK_BusClk)/32)
#define SERVO_PWM_FREQ (50.0f) /* Hz */
#define SEC_2_US (1000000)
#define SERVO_PERIOD (uint16_t)((1/SERVO_PWM_FREQ)*SEC_2_US)

#define MIN_PULSE_TIME   (uint16_t)(1000) /* us */
#define MAX_PULSE_WIDTH  (uint16_t)(1000) /* us */

#define MIN_ANGLE_DEG (0)
#define DEF_ANGLE_DEG (90)
#define MAX_ANGLE_DEG (180)

#define ANGLE_2_PULSE_TIME(angle) (uint16_t)((angle*(MAX_PULSE_WIDTH/MAX_ANGLE_DEG))+MIN_PULSE_TIME)

static volatile Software_PWM_T PWM;
static volatile uint16_t test;

void Servo::Init(void)
{
   /* Configure the PWM output */
   ftm_config_t ftmInfo;

   /* Initialize the software based PWM parameters */
   PWM.period   = SERVO_PERIOD;
   PWM.on_time  = ANGLE_2_PULSE_TIME(DEF_ANGLE_DEG);
   PWM.off_time = (PWM.period - PWM.on_time);
   PWM.dc_state = DC_ON;

   FTM_GetDefaultConfig(&ftmInfo);

   /* Divide FTM clock by 32 */
   ftmInfo.prescale = kFTM_Prescale_Divide_32;

   FTM_Init(FTM3, &ftmInfo);

   FTM_SetTimerPeriod(FTM3, USEC_TO_COUNT(PWM.on_time, FTM_SOURCE_CLOCK));

   FTM_EnableInterrupts(FTM3, kFTM_TimeOverflowInterruptEnable);

   EnableIRQ(FTM3_IRQn);

   Set_GPIO(SERVO, HIGH);

   FTM_StartTimer(FTM3, kFTM_SystemClock);

   init_complete = true;
}

void Servo::Set_Postion(uint8_t angle)
{
   if (!init_complete)
   {
      /* Init method must be called first */
      assert(false);
   }

   if (angle > (uint8_t) MAX_ANGLE_DEG)
   {
      /* Servo can only rotate 180 degrees */
      assert(false);
   }

   FTM_DisableInterrupts(FTM3, kFTM_TimeOverflowInterruptEnable);
   FTM_StopTimer(FTM3);

   PWM.on_time = ANGLE_2_PULSE_TIME(angle);
   PWM.off_time = (PWM.period - PWM.on_time);

   FTM_SetTimerPeriod(FTM3, USEC_TO_COUNT(PWM.on_time, FTM_SOURCE_CLOCK));
   PWM.dc_state = DC_ON;
   Set_GPIO(SERVO, HIGH);

   FTM_StartTimer(FTM3, kFTM_SystemClock);
   FTM_EnableInterrupts(FTM3, kFTM_TimeOverflowInterruptEnable);
}

extern "C"
{
void FTM3_IRQHandler(void)
{
   if (DC_ON == PWM.dc_state)
   {
      FTM_StopTimer(FTM3);
      FTM_SetTimerPeriod(FTM3, USEC_TO_COUNT(PWM.off_time, FTM_SOURCE_CLOCK));
      PWM.dc_state = DC_OFF;
      Set_GPIO(SERVO, LOW);
      FTM_StartTimer(FTM3, kFTM_SystemClock);
   }
   else
   {
      FTM_StopTimer(FTM3);
      FTM_SetTimerPeriod(FTM3, USEC_TO_COUNT(PWM.on_time, FTM_SOURCE_CLOCK));
      PWM.dc_state = DC_ON;
      Set_GPIO(SERVO, HIGH);
      FTM_StartTimer(FTM3, kFTM_SystemClock);
   }

   /* Clear interrupt flag.*/
    FTM_ClearStatusFlags(FTM3, kFTM_TimeOverflowFlag);
    __DSB();
}
}
