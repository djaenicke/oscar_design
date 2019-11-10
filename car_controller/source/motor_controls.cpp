/*
 * motor_controls.cpp
 *
 *  Created on: Jun 7, 2019
 *      Author: Devin
 */

#include <stdio.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "dc_motor.h"
#include "behaviors.h"
#include "inertial_states.h"
#include "fsl_debug_console.h"
#include "pid.h"
#include "clock_config.h"
#include "wheel_speeds.h"
#include "battery_monitor.h"
#include "low_pass_filter.h"
#include "logging_streams.h"
#include "debug_constants.h"
#include "assert.h"

#define NUM_MOTORS 2
#define FTM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)
#define PWM_FREQ 1000U /* Hz */

#define R_Ke 0.226f
#define L_Ke 0.192f
#define DRIVER_VDROP 2.0f

//#define OPEN_LOOP
#define L_Kp 6.0f
#define L_Ki 26.0f
#define L_Kd 0.0034531f

#define R_Kp 6.0f
#define R_Ki 18.0f
#define R_Kd 0.0034531f

#define TOLERANCE 0.0f /* rad/s - TODO: update*/

#define VBATT_FILT_ALPHA       0.4f

/* Motor objects */
static DC_Motor L_Motor;
static DC_Motor R_Motor;

/* PID controller objects */
static PID L_PID;
static PID R_PID;

static const float Min_Voltage = 3.0; /* Motors require at least 3.0V */

static Motor_Controls_Stream_T MC_Stream_Data;

void Init_Motor_Controls(void)
{
   /* Configure the PWM outputs to allow speed control */
   ftm_config_t ftmInfo;
   ftm_chnl_pwm_signal_param_t ftmParam[NUM_MOTORS];
   PID_Cals_T pid_cals;

   L_Motor.Set_Location(LEFT_SIDE);
   R_Motor.Set_Location(RIGHT_SIDE);

   pid_cals.dt  = CYCLE_TIME;
   pid_cals.tol = TOLERANCE;

   pid_cals.k_p = L_Kp;
   pid_cals.k_i = L_Ki;
   pid_cals.k_d = L_Kd;
   L_PID.Init(&pid_cals);

   pid_cals.k_p = R_Kp;
   pid_cals.k_i = R_Ki;
   pid_cals.k_d = R_Kd;
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

void Run_Motor_Controls(void)
{
   size_t bytes_sent = 0;
   float v_r_sp, v_l_sp;   /* Set point */
   float u_r, u_l; /* Actuation signals */
   Direction_T r_dir, l_dir;
#if 0 == OPEN_LOOP
   float v_r_fb, v_l_fb;   /* Feedback */
#else
   int8_t sign;
#endif

   MC_Stream_Data.cnt++;
   Get_Wheel_Ang_Velocities(&MC_Stream_Data.wheel_ang_v);

#if TUNE
   /* Used for tuning the PID controllers */
   uint16_t sp_debug = (uint16_t)(MC_Stream_Data.r_ang_v_sp*1000);
   uint16_t r_debug  = (uint16_t)(MC_Stream_Data.wheel_ang_v.r*1000);
   uint16_t l_debug  = (uint16_t)(MC_Stream_Data.wheel_ang_v.l*1000);

   PRINTF("%d,%d,%d,%d\n\r", MC_Stream_Data.cnt, sp_debug, r_debug, l_debug);
#endif

   /* Determine the maximum actuation voltage based on the current battery voltage */
   MC_Stream_Data.meas_vbatt = LP_Filter(Read_Battery_Voltage(), MC_Stream_Data.meas_vbatt, VBATT_FILT_ALPHA);
   MC_Stream_Data.max_vbatt  = MC_Stream_Data.meas_vbatt - DRIVER_VDROP;

   if (!L_Motor.stopped && !R_Motor.stopped)
   {
      /* Compute the voltage set points */
      v_r_sp = MC_Stream_Data.r_ang_v_sp * R_Ke;
      v_l_sp = MC_Stream_Data.l_ang_v_sp * L_Ke;
#if 0 == OPEN_LOOP
      /* Compute the voltage feedback */
      v_r_fb = MC_Stream_Data.wheel_ang_v.r * R_Ke;
      v_l_fb = MC_Stream_Data.wheel_ang_v.l * L_Ke;

      /* Run the PID controllers */
      u_r = R_PID.Step(v_r_sp, v_r_fb, MC_Stream_Data.max_vbatt, Min_Voltage);
      u_l = L_PID.Step(v_l_sp, v_l_fb, MC_Stream_Data.max_vbatt, Min_Voltage);
#else
      /* Saturate the set points to be within the actuator voltage range */
      sign = signbit(v_r_sp) ? -1 : 1;
      u_r  = fabs(v_r_sp) > Min_Voltage ? v_r_sp : sign * Min_Voltage;
      u_r  = fabs(v_r_sp) < MC_Stream_Data.max_vbatt ? v_r_sp : sign * MC_Stream_Data.max_vbatt;

      sign = signbit(v_l_sp) ? -1 : 1;
      u_l  = fabs(v_l_sp) > Min_Voltage ? v_l_sp : sign * Min_Voltage;
      u_l  = fabs(v_l_sp) < MC_Stream_Data.max_vbatt ? v_l_sp : sign * MC_Stream_Data.max_vbatt;
#endif
      /* Convert the actuation voltages to duty cycles */
      MC_Stream_Data.u_r_dc = (uint8_t)(fabs(u_r) * (100/MC_Stream_Data.max_vbatt));
      MC_Stream_Data.u_l_dc = (uint8_t)(fabs(u_l) * (100/MC_Stream_Data.max_vbatt));

      /* Determine direction */
      r_dir = signbit(u_r) ? REVERSE : FORWARD;
      l_dir = signbit(u_l) ? REVERSE : FORWARD;

      R_Motor.Set_Direction(r_dir);
      L_Motor.Set_Direction(l_dir);

      /* Zero the wheel speeds on a direction change */
      if (r_dir != R_Motor.Get_Direction())
      {
         Zero_Wheel_Speed(R);
      }
      if (l_dir != L_Motor.Get_Direction())
      {
         Zero_Wheel_Speed(L);
      }
   }
   else
   {
      MC_Stream_Data.u_r_dc = 0;
      MC_Stream_Data.u_l_dc = 0;
   }

   R_Motor.Set_DC(MC_Stream_Data.u_r_dc);
   L_Motor.Set_DC(MC_Stream_Data.u_l_dc);

   bytes_sent = xStreamBufferSend(MC_Stream.handle, (void *) &MC_Stream_Data, sizeof(MC_Stream_Data), 0);
   assert(bytes_sent == sizeof(MC_Stream_Data));
}

bool Right_Motor_Stopped(void)
{
   return(R_Motor.stopped);
}

bool Left_Motor_Stopped(void)
{
   return(L_Motor.stopped);
}

int8_t Right_Wheel_Speed_Sign(void)
{
   return(FORWARD == R_Motor.Get_Direction() ? 1 : -1);
}

int8_t Left_Wheel_Speed_Sign(void)
{
   return(FORWARD == L_Motor.Get_Direction() ? 1 : -1);
}


void Update_Wheel_Angular_V_SP(float l_sp, float r_sp, bool reset_pid)
{
   MC_Stream_Data.r_ang_v_sp = r_sp;
   MC_Stream_Data.l_ang_v_sp = l_sp;

   R_Motor.stopped = false;
   L_Motor.stopped = false;

   if (reset_pid)
   {
      R_PID.Reset();
      L_PID.Reset();
   }
}

void Stop(void)
{
   Update_Wheel_Angular_V_SP(0.0f, 0.0f, true);
   L_Motor.Stop();
   R_Motor.Stop();
}

