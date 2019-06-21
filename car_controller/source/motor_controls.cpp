/*
 * motor_controls.cpp
 *
 *  Created on: Jun 7, 2019
 *      Author: Devin
 */

#include "dc_motor.h"
#include "clock_config.h"
#include "wheel_speeds.h"
#include "battery_monitor.h"

#include "FreeRTOS.h"
#include "task.h"

#define NUM_MOTORS 2
#define FTM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)
#define PWM_FREQ 1000U /* Hz */

#define Kv 4.54f
#define Ke 0.22f
#define DRIVER_VDROP 2.0f

/* Motor objects */
static DC_Motor L_Motor;
static DC_Motor R_Motor;


static const float Min_Voltage = 3.0;         /* Motors require at least 3.0V */
static float Max_Voltage;                     /* Max voltage is dependent on the battery SoC */

static Wheel_Speeds_T Wheel_Speeds;
static float R_Motor_Speed;
static float L_Motor_Speed;

void Init_Motor_Controls(void)
{
   /* Configure the PWM outputs to allow speed control */
   ftm_config_t ftmInfo;
   ftm_chnl_pwm_signal_param_t ftmParam[NUM_MOTORS];

   L_Motor.Set_Location(LEFT_SIDE);
   R_Motor.Set_Location(RIGHT_SIDE);

   /* Left Side */
   ftmParam[0].chnlNumber = L_Motor.pwm_channel;
   ftmParam[0].level = kFTM_HighTrue;
   ftmParam[0].dutyCyclePercent = 0U;
   ftmParam[0].firstEdgeDelayPercent = 0U;

   /* Right Side */
   ftmParam[1].chnlNumber = R_Motor.pwm_channel;
   ftmParam[1].level = kFTM_HighTrue;
   ftmParam[1].dutyCyclePercent = 0U;
   ftmParam[1].firstEdgeDelayPercent = 0U;

   FTM_GetDefaultConfig(&ftmInfo);

   /* Initialize FTM module */
   FTM_Init(FTM0, &ftmInfo);

   FTM_SetupPwm(FTM0, ftmParam, NUM_MOTORS, kFTM_EdgeAlignedPwm, PWM_FREQ, FTM_SOURCE_CLOCK);
   FTM_StartTimer(FTM0, kFTM_SystemClock);
}

void Motor_Controls_Task(void *pvParameters)
{
   float vbatt;

   while(1)
   {
      //Get_Wheel_Speeds(&Wheel_Speeds); TODO: uncomment this

      /* Average the wheel speeds to treat the 4 motors as 2 */
      R_Motor_Speed = (Wheel_Speeds.rr + Wheel_Speeds.fr)/2;
      L_Motor_Speed = (Wheel_Speeds.rl + Wheel_Speeds.fl)/2;

      vbatt = Read_Battery_Voltage();
      Max_Voltage = vbatt - DRIVER_VDROP;

      vTaskDelay(pdMS_TO_TICKS(1000)); //TODO: change to 25
   }
}

void Forward(void)
{
   L_Motor.Set_DC(100);
   R_Motor.Set_DC(100);
   L_Motor.Set_Direction(FORWARD);
   R_Motor.Set_Direction(FORWARD);
}

void Backward(void)
{
   L_Motor.Set_DC(100);
   R_Motor.Set_DC(100);
   L_Motor.Set_Direction(REVERSE);
   R_Motor.Set_Direction(REVERSE);
}

void Left(void)
{
   L_Motor.Set_DC(60);
   R_Motor.Set_DC(60);
   L_Motor.Set_Direction(REVERSE);
   R_Motor.Set_Direction(FORWARD);
}

void Right(void)
{
   L_Motor.Set_DC(60);
   R_Motor.Set_DC(60);
   L_Motor.Set_Direction(FORWARD);
   R_Motor.Set_Direction(REVERSE);
}

void Stop(void)
{
   L_Motor.Stop();
   R_Motor.Stop();
}

