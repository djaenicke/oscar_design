/*
 * inertial_states.cpp
 *
 *  Created on: Jul 7, 2019
 *      Author: Devin
 */

#include "inertial_states.h"
#include "motor_controls.h"
#include "mpu6050.h"
#include "FXOS8700CQ.h"
#include "low_pass_filter.h"

#define WHEEL_SPEED_FILT_ALPHA 0.4f

static MPU6050    My_MPU6050;
static FXOS8700CQ My_FXOS8700CQ;

static Accel_Data_T MPU6050_Accel_Data = {0};
static Gyro_Data_T  MPU6050_Gyro_Data  = {0};

static Wheel_Speeds_T Raw_Wheel_Ang_V = {0};
static Wheel_Speeds_T Filt_Wheel_Ang_V = {0};

static inline void Sample_Wheel_Velocities(void);
static inline void Convert_Speeds_2_Velocities(Wheel_Speeds_T * speeds);
static inline void Filter_Wheel_Velocities(void);

void Init_Inertial_Sensors(void)
{
   /* Initialize the external 6-axis MPU6050 */
   My_MPU6050.Init(FTM0, I2C1);

   /* Initialize the 6-axis on board FXOS8700CQ */
   My_FXOS8700CQ.Init();

   /* Get readings from MPU6050 sensor */
   My_MPU6050.Read_Accel_Data(&MPU6050_Accel_Data);
   My_MPU6050.Read_Gyro_Data(&MPU6050_Gyro_Data);
}

void Update_Robot_States(void)
{
   Sample_Wheel_Velocities();
   My_MPU6050.Read_Accel_Data(&MPU6050_Accel_Data);
   My_MPU6050.Read_Gyro_Data(&MPU6050_Gyro_Data);
}

void Sample_Wheel_Velocities(void)
{
   Wheel_Speeds_T wheel_speeds = {0};

   if (Right_Motor_Stopped() && Left_Motor_Stopped())
   {
      Zero_Wheel_Speed(R_HE);
      Zero_Wheel_Speed(R_E);
      Zero_Wheel_Speed(L_HE);
      Zero_Wheel_Speed(L_E);
   }

   Get_Wheel_Speeds(&wheel_speeds);
   Convert_Speeds_2_Velocities(&wheel_speeds);
   Filter_Wheel_Velocities();
}

static inline void Convert_Speeds_2_Velocities(Wheel_Speeds_T * speeds)
{
   int8_t sign;

   sign = Right_Wheel_Speed_Sign();
   Raw_Wheel_Ang_V.r    = sign * speeds->r;
   Raw_Wheel_Ang_V.r_he = sign * speeds->r_he;

   sign = Left_Wheel_Speed_Sign();
   Raw_Wheel_Ang_V.l    = sign * speeds->l;
   Raw_Wheel_Ang_V.l_he = sign * speeds->l_he;
}

static inline void Filter_Wheel_Velocities()
{
   Filt_Wheel_Ang_V.r = LP_Filter(Raw_Wheel_Ang_V.r, Filt_Wheel_Ang_V.r, WHEEL_SPEED_FILT_ALPHA);
   Filt_Wheel_Ang_V.l = LP_Filter(Raw_Wheel_Ang_V.l, Filt_Wheel_Ang_V.l, WHEEL_SPEED_FILT_ALPHA);

   Filt_Wheel_Ang_V.r_he = LP_Filter(Raw_Wheel_Ang_V.r_he, Filt_Wheel_Ang_V.r_he, WHEEL_SPEED_FILT_ALPHA);
   Filt_Wheel_Ang_V.l_he = LP_Filter(Raw_Wheel_Ang_V.l_he, Filt_Wheel_Ang_V.l_he, WHEEL_SPEED_FILT_ALPHA);
}

void Get_Wheel_Ang_Velocities(Wheel_Speeds_T * ang_velocities)
{
   assert(ang_velocities);
   (void) memcpy(ang_velocities, &Filt_Wheel_Ang_V, sizeof(Wheel_Speeds_T));
}

