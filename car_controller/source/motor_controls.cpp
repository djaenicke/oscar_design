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
#include "fsl_debug_console.h"
#include "pid.h"
#include "clock_config.h"
#include "wheel_speeds.h"
#include "battery_monitor.h"
#include "low_pass_filter.h"
#include "logging_streams.h"
#include "assert.h"

//#define OPEN_LOOP
//#define DEBUG

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

#define CYCLE_TIME 0.025f
#define S_2_MS 1000

#define VBATT_FILT_ALPHA       0.4f
#define WHEEL_SPEED_FILT_ALPHA 0.4f

/* Motor objects */
static DC_Motor L_Motor;
static DC_Motor R_Motor;

/* PID controller objects */
static PID L_PID;
static PID R_PID;

static const float Min_Voltage = 3.0; /* Motors require at least 3.0V */

static Motor_Controls_Stream_T MC_Stream_Data;

static inline void Filter_Wheel_Speeds(void);
static inline void Convert_Speeds_2_Velocities(void);

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

void Motor_Controls_Task(void *pvParameters)
{
   size_t bytes_sent = 0;
   float v_r_sp, v_l_sp;   /* Set point */
   Direction_T r_dir, l_dir;
#ifndef OPEN_LOOP
   float v_r_fb, v_l_fb;   /* Feedback */
#else
   int8_t sign;
#endif
   while(1)
   {
      MC_Stream_Data.cnt++;

      if (L_Motor.stopped && R_Motor.stopped)
      {
         Zero_Wheel_Speed(R_HE);
         Zero_Wheel_Speed(R_E);
         Zero_Wheel_Speed(L_HE);
         Zero_Wheel_Speed(L_E);
      }

      Get_Wheel_Speeds(&MC_Stream_Data.raw_speeds);
      Convert_Speeds_2_Velocities();
      Filter_Wheel_Speeds();

#ifdef DEBUG
      /* Used for tuning the PID controllers */
      uint16_t sp_debug = (uint16_t)(MC_Stream_Data.r_speed_sp*1000);
      uint16_t r_debug  = (uint16_t)(MC_Stream_Data.filt_speeds.r_he*1000);
      uint16_t l_debug  = (uint16_t)(MC_Stream_Data.filt_speeds.l_he*1000);

      PRINTF("%d,%d,%d,%d\n\r", MC_Stream_Data.cnt, sp_debug, r_debug, l_debug);
#endif

      /* Determine the maximum actuation voltage based on the current battery voltage */
      MC_Stream_Data.meas_vbatt = LP_Filter(Read_Battery_Voltage(), MC_Stream_Data.meas_vbatt, VBATT_FILT_ALPHA);
      MC_Stream_Data.max_vbatt  = MC_Stream_Data.meas_vbatt - DRIVER_VDROP;

      if (!L_Motor.stopped && !R_Motor.stopped)
      {
         /* Compute the voltage set points */
         v_r_sp = MC_Stream_Data.r_speed_sp * R_Ke;
         v_l_sp = MC_Stream_Data.l_speed_sp * L_Ke;
#ifndef OPEN_LOOP
         /* Compute the voltage feedback */
         v_r_fb = MC_Stream_Data.filt_speeds.r_he * R_Ke;
         v_l_fb = MC_Stream_Data.filt_speeds.l_he * L_Ke;

         /* Run the PID controllers */
         MC_Stream_Data.u_r = R_PID.Step(v_r_sp, v_r_fb, MC_Stream_Data.max_vbatt, Min_Voltage);
         MC_Stream_Data.u_l = L_PID.Step(v_l_sp, v_l_fb, MC_Stream_Data.max_vbatt, Min_Voltage);

         /* Get debug information for data logging */
         MC_Stream_Data.r_error    = R_PID.last_e;
         MC_Stream_Data.l_error    = L_PID.last_e;
         MC_Stream_Data.r_integral = R_PID.integral;
         MC_Stream_Data.l_integral = L_PID.integral;
#else
         /* Saturate the set points to be within the actuator voltage range */
         sign = signbit(MC_Stream_Data.u_r) ? -1 : 1;
         MC_Stream_Data.u_r = fabs(v_r_sp) > Min_Voltage ? v_r_sp : sign * Min_Voltage;
         MC_Stream_Data.u_r = fabs(v_r_sp) < MC_Stream_Data.max_vbatt ? v_r_sp : sign * MC_Stream_Data.max_vbatt;

         sign = signbit(MC_Stream_Data.u_l) ? -1 : 1;
         MC_Stream_Data.u_l = fabs(v_l_sp) > Min_Voltage ? v_l_sp : sign *Min_Voltage;
         MC_Stream_Data.u_l = fabs(v_l_sp) < MC_Stream_Data.max_vbatt ? v_l_sp : sign * MC_Stream_Data.max_vbatt;
#endif
         /* Convert the actuation voltages to duty cycles */
         MC_Stream_Data.u_r_dc = (uint8_t)(fabs(MC_Stream_Data.u_r) * (100/MC_Stream_Data.max_vbatt));
         MC_Stream_Data.u_l_dc = (uint8_t)(fabs(MC_Stream_Data.u_l) * (100/MC_Stream_Data.max_vbatt));

         /* Determine direction */
         r_dir = signbit(MC_Stream_Data.u_r) ? REVERSE : FORWARD;
         l_dir = signbit(MC_Stream_Data.u_l) ? REVERSE : FORWARD;

         R_Motor.Set_Direction(r_dir);
         L_Motor.Set_Direction(l_dir);

         /* Zero the wheel speeds on a direction change */
         if (r_dir != R_Motor.Get_Direction())
         {
            Zero_Wheel_Speed(R_HE);
            Zero_Wheel_Speed(R_E);
         }
         if (l_dir != L_Motor.Get_Direction())
         {
            Zero_Wheel_Speed(L_HE);
            Zero_Wheel_Speed(L_E);
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

      vTaskDelay(pdMS_TO_TICKS(CYCLE_TIME*S_2_MS));
   }
}

static inline void Filter_Wheel_Speeds(void)
{
   MC_Stream_Data.filt_speeds.r = LP_Filter(MC_Stream_Data.raw_speeds.r,
                                            MC_Stream_Data.filt_speeds.r,
                                            WHEEL_SPEED_FILT_ALPHA);

   MC_Stream_Data.filt_speeds.l = LP_Filter(MC_Stream_Data.raw_speeds.l,
                                            MC_Stream_Data.filt_speeds.l,
                                            WHEEL_SPEED_FILT_ALPHA);

   MC_Stream_Data.filt_speeds.r_he = LP_Filter(MC_Stream_Data.raw_speeds.r_he,
                                               MC_Stream_Data.filt_speeds.r_he,
                                               WHEEL_SPEED_FILT_ALPHA);

   MC_Stream_Data.filt_speeds.l_he = LP_Filter(MC_Stream_Data.raw_speeds.l_he,
                                               MC_Stream_Data.filt_speeds.l_he,
                                               WHEEL_SPEED_FILT_ALPHA);
}

static inline void Convert_Speeds_2_Velocities(void)
{
   int8_t sign;

   sign = FORWARD == R_Motor.Get_Direction() ? 1 : -1;
   MC_Stream_Data.raw_speeds.r = sign * MC_Stream_Data.raw_speeds.r;

   sign = FORWARD == L_Motor.Get_Direction() ? 1 : -1;
   MC_Stream_Data.raw_speeds.l = sign * MC_Stream_Data.raw_speeds.l;
}

void Update_Wheel_Speed_Setpoints(float l_sp, float r_sp)
{
   MC_Stream_Data.r_speed_sp = r_sp;
   MC_Stream_Data.l_speed_sp = l_sp;

   R_Motor.stopped = false;
   L_Motor.stopped = false;

   R_PID.Reset();
   L_PID.Reset();
}

void Stop(void)
{
   Update_Wheel_Speed_Setpoints(0.0f, 0.0f);
   L_Motor.Stop();
   R_Motor.Stop();
}

