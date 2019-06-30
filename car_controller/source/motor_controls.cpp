/*
 * motor_controls.cpp
 *
 *  Created on: Jun 7, 2019
 *      Author: Devin
 */

#include "FreeRTOS.h"
#include "task.h"

#include "dc_motor.h"
#include "pid.h"
#include "clock_config.h"
#include "wheel_speeds.h"
#include "battery_monitor.h"
#include "low_pass_filter.h"
#include "logging_streams.h"
#include "assert.h"

#define NUM_MOTORS 2
#define FTM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)
#define PWM_FREQ 1000U /* Hz */

#define Kv 4.54f
#define Ke 0.22f
#define DRIVER_VDROP 2.0f

/* TODO - tune these */
#define OPEN_LOOP
#define Kp 1.0f
#define Ki 1.0f
#define Kd 0.0f

#define dt 0.025
#define S_2_MS 1000

#define FILT_ALPHA 0.4f

/* Motor objects */
static DC_Motor L_Motor;
static DC_Motor R_Motor;

/* PID controller objects */
static PID L_PID;
static PID R_PID;

static const float Min_Voltage = 3.0; /* Motors require at least 3.0V */

static Motor_Controls_Stream_T MC_Stream_Data;

static inline void Filter_Wheel_Speeds(void);

void Init_Motor_Controls(void)
{
   /* Configure the PWM outputs to allow speed control */
   ftm_config_t ftmInfo;
   ftm_chnl_pwm_signal_param_t ftmParam[NUM_MOTORS];
   PID_Cals_T pid_cals;

   L_Motor.Set_Location(LEFT_SIDE);
   R_Motor.Set_Location(RIGHT_SIDE);

   L_PID.Init(&pid_cals);
   R_PID.Init(&pid_cals);

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

   MC_Stream_Data.end_pattern = END_PATTERN;
}

void Motor_Controls_Task(void *pvParameters)
{
   size_t bytes_sent;

   while(1)
   {
      MC_Stream_Data.cnt++;

      if (L_Motor.stopped && R_Motor.stopped)
      {
         Zero_Wheel_Speeds();
      }

      Get_Wheel_Speeds(&MC_Stream_Data.raw_speeds);
      Filter_Wheel_Speeds();

      /* Average the wheel speeds to treat the 4 motors as 2 */
      MC_Stream_Data.r_speed_fb = (MC_Stream_Data.filt_speeds.rr + MC_Stream_Data.filt_speeds.fr)/2;
      MC_Stream_Data.l_speed_fb = (MC_Stream_Data.filt_speeds.rl + MC_Stream_Data.filt_speeds.fl)/2;

      MC_Stream_Data.meas_vbatt = Read_Battery_Voltage();
      MC_Stream_Data.max_vbatt = MC_Stream_Data.meas_vbatt - DRIVER_VDROP;

#ifndef OPEN_LOOP
      R_PID.Step(???, MC_Stream_Data.r_speed_fb, dt);
      L_PID.Step(???, MC_Stream_Data.l_speed_fb, dt);
#endif

      bytes_sent = xStreamBufferSend(MC_Stream.handle, (void *) &MC_Stream_Data, sizeof(MC_Stream_Data), 0);
      assert(bytes_sent == sizeof(MC_Stream_Data));

      vTaskDelay(pdMS_TO_TICKS(dt*S_2_MS));
   }
}

static inline void Filter_Wheel_Speeds(void)
{
   MC_Stream_Data.filt_speeds.rr = LP_Filter(MC_Stream_Data.raw_speeds.rr, MC_Stream_Data.filt_speeds.rr, FILT_ALPHA);
   MC_Stream_Data.filt_speeds.rl = LP_Filter(MC_Stream_Data.raw_speeds.rl, MC_Stream_Data.filt_speeds.rl, FILT_ALPHA);
   MC_Stream_Data.filt_speeds.fr = LP_Filter(MC_Stream_Data.raw_speeds.fr, MC_Stream_Data.filt_speeds.fr, FILT_ALPHA);
   MC_Stream_Data.filt_speeds.fl = LP_Filter(MC_Stream_Data.raw_speeds.fl, MC_Stream_Data.filt_speeds.fl, FILT_ALPHA);
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

