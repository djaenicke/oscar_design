/*
 * go_to_point.cpp
 *
 *  Created on: Jul 28, 2019
 *      Author: Devin
 */

#include <math.h>

#include "go_to_point.h"
#include "motor_controls.h"
#include "fsl_debug_console.h"
#include "assert.h"

/* Controller Design */
/* https://pdfs.semanticscholar.org/edde/fa921e26efbbfd6c65ad1e13af0bbbc1b946.pdf */

void GoToPointController::Init(float tolerance, float gain, float travel_s)
{
   tol = tolerance;
   kp  = gain;
   robot_v = travel_s;
   in_route = false;
}

void GoToPointController::Execute(void)
{
   static uint32_t cnt;
   float vr, vl; /* Desired linear wheel velocities */
   float theta_d, theta_e; /* Heading difference and heading error */
   float omega;
   float d; /* distance to destination */

   /* Get the heading and X and Y positions */
   Get_Pose(&pose_fb);

   /* Compute the distance to the objective */
   d = sqrt(pow(pose_sp.x - pose_fb.x, 2) + pow(pose_sp.y - pose_fb.y, 2));

   if (d > tol)
   {
      theta_d = pose_sp.theta - pose_fb.theta;        /* Heading difference */
      theta_e = atan2f(sinf(theta_d), cosf(theta_d)); /* Heading error */

      /* Proportional control */
      omega = kp * theta_e;

      /* Determine the required linear velocities */
      vr = robot_v + (omega*WHEEL_BASE/2);
      vl = robot_v - (omega*WHEEL_BASE/2);

#if 1
      PRINTF("%d,%d,%d,%d,%d,%d,%d,%d,%d\n\r", cnt++, (int16_t)(pose_sp.x*1000), (int16_t)(pose_sp.y*1000), (int16_t)(pose_sp.theta*1000), \
                                                      (int16_t)(pose_fb.x*1000), (int16_t)(pose_fb.y*1000), (int16_t)(pose_fb.theta*1000), \
                                                      (int16_t)(vr*1000), (int16_t)(vl*1000));
#endif
      Update_Wheel_Angular_V_SP(vl/WHEEL_RADIUS, vr/WHEEL_RADIUS, false);
   }
   else
   {
      /* Within tol of destination */
      PRINTF("Destination reached\n\r");
      Stop();
      in_route = false;
   }
}

void GoToPointController::Set_Travel_Speed(float robot_v)
{
   /* Linear velocity in (m/s) */
   robot_v = robot_v;
}

void GoToPointController::Update_Destination(Destination_T * dest)
{
   assert(dest);

   /* Compute the go-to-point controller set point */
   pose_sp.x = fabsf(dest->x);
   pose_sp.y = fabsf(dest->y);
   pose_sp.theta = (0-pose_fb.theta) + atan2f(dest->y, dest->x);
   in_route = true;

   Reset_Inertial_Data();
}

bool GoToPointController::In_Route(void)
{
   return (in_route);
}

