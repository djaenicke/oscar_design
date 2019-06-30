/*
 * motor_controls.cpp
 *
 *  Created on: Jun 7, 2019
 *      Author: Devin
 */

#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "dc_motor.h"
#include "fsl_debug_console.h"
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

//#define OPEN_LOOP
#define Kp 0.2762472f
#define Ki 5.5249447f
#define Kd 0.0034531f

#define CYCLE_TIME 0.05f
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


   pid_cals.k_p = Kp;
   pid_cals.k_i = Ki;
   pid_cals.k_d = Kd;
   pid_cals.dt  = CYCLE_TIME;

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
   float v_r_sp, v_l_sp;   /* Set point */
   uint8_t u_r_dc, u_l_dc; /* Actuation duty cycles */
#ifndef OPEN_LOOP
   float v_r_fb, v_l_fb;   /* Feedback */
   float u_r, u_l;         /* Actuation voltages */
#endif
   while(1)
   {
      MC_Stream_Data.cnt++;

      if (L_Motor.stopped && R_Motor.stopped)
      {
         Zero_Wheel_Speeds();
      }

      Get_Wheel_Speeds(&MC_Stream_Data.raw_speeds);
      Filter_Wheel_Speeds();

      /* Determine the maximum speed based on the current battery voltage */
      MC_Stream_Data.meas_vbatt = Read_Battery_Voltage();
      MC_Stream_Data.max_vbatt  = MC_Stream_Data.meas_vbatt - DRIVER_VDROP;
      MC_Stream_Data.max_speed  = MC_Stream_Data.max_vbatt * Kv;

      /* Saturate the speed set point */
      MC_Stream_Data.r_speed_sp = MC_Stream_Data.r_speed_sp > MC_Stream_Data.max_speed ? MC_Stream_Data.max_speed : MC_Stream_Data.r_speed_sp;
      MC_Stream_Data.l_speed_sp = MC_Stream_Data.l_speed_sp > MC_Stream_Data.max_speed ? MC_Stream_Data.max_speed : MC_Stream_Data.l_speed_sp;

      /* Compute the set points */
      v_r_sp = MC_Stream_Data.r_speed_sp * Ke;
      v_l_sp = MC_Stream_Data.l_speed_sp * Ke;

      /* Average the wheel speeds to treat the 4 motors as 2 */
      MC_Stream_Data.r_speed_fb = (MC_Stream_Data.filt_speeds.rr + MC_Stream_Data.filt_speeds.fr)/2;
      MC_Stream_Data.l_speed_fb = (MC_Stream_Data.filt_speeds.rl + MC_Stream_Data.filt_speeds.fl)/2;

#ifndef OPEN_LOOP
      /* Compute the feedback */
      v_r_fb = MC_Stream_Data.r_speed_fb * Ke;
      v_l_fb = MC_Stream_Data.r_speed_fb * Ke;

      /* Run the PID controllers */
      u_r = R_PID.Step(v_r_sp, v_r_fb);
      u_l = L_PID.Step(v_l_sp, v_l_fb);

      u_r = u_r > Min_Voltage ? u_r : Min_Voltage;
      u_l = u_l > Min_Voltage ? u_l : Min_Voltage;

      /* Convert the voltages to duty cycles */
      u_r_dc = (uint8_t)(u_r * (100/MC_Stream_Data.max_vbatt));
      u_l_dc = (uint8_t)(u_l * (100/MC_Stream_Data.max_vbatt));
#endif

#ifdef OPEN_LOOP
      /* Convert the voltages to duty cycles */
      v_r_sp = v_r_sp > Min_Voltage ? v_r_sp : Min_Voltage;
      v_l_sp = v_l_sp > Min_Voltage ? v_l_sp : Min_Voltage;

      u_r_dc = (uint8_t)(v_r_sp * (100/MC_Stream_Data.max_vbatt));
      u_l_dc = (uint8_t)(v_l_sp * (100/MC_Stream_Data.max_vbatt));
#endif

      L_Motor.Set_DC(u_r_dc);
      R_Motor.Set_DC(u_l_dc);

      bytes_sent = xStreamBufferSend(MC_Stream.handle, (void *) &MC_Stream_Data, sizeof(MC_Stream_Data), 0);
      assert(bytes_sent == sizeof(MC_Stream_Data));

      vTaskDelay(pdMS_TO_TICKS(CYCLE_TIME*S_2_MS));
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
   MC_Stream_Data.r_speed_sp = 20;
   MC_Stream_Data.l_speed_sp = 20;

   L_Motor.Set_Direction(FORWARD);
   R_Motor.Set_Direction(FORWARD);
}

void Backward(void)
{
   MC_Stream_Data.r_speed_sp = 20;
   MC_Stream_Data.l_speed_sp = 20;

   L_Motor.Set_Direction(REVERSE);
   R_Motor.Set_Direction(REVERSE);
}

void Left(void)
{
   MC_Stream_Data.r_speed_sp = 17;
   MC_Stream_Data.l_speed_sp = 17;

   L_Motor.Set_Direction(REVERSE);
   R_Motor.Set_Direction(FORWARD);
}

void Right(void)
{
   MC_Stream_Data.r_speed_sp = 17;
   MC_Stream_Data.l_speed_sp = 17;

   L_Motor.Set_Direction(FORWARD);
   R_Motor.Set_Direction(REVERSE);
}

void Stop(void)
{
   L_Motor.Stop();
   R_Motor.Stop();
}

