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
#include "constants.h"

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
#if DEBUG_GTP_CONTROLLER
   static uint32_t cnt;
#endif
   float vr, vl; /* Desired linear wheel velocities */
   float theta_d, theta_e; /* Heading difference and heading error */
   float omega;
   float d; /* distance to destination */

   /* Get the heading and X and Y positions */
   Get_Pose(&pose_fb);

   /* Compute the distance traveled */
   d = sqrt(pow(pose_fb.x, 2) + pow(pose_fb.y, 2));

   if ((d_sp - d) > tol)
   {
      theta_d = heading_sp - pose_fb.theta;           /* Heading difference */
      theta_e = atan2f(sinf(theta_d), cosf(theta_d)); /* Heading error      */

      /* Proportional control */
      omega = kp * theta_e;

      /* Determine the required linear velocities */
      vr = robot_v + (omega*WHEEL_BASE/2);
      vl = robot_v - (omega*WHEEL_BASE/2);

#if DEBUG_GTP_CONTROLLER
      PRINTF("%d,%.2f,%.2f,%.2f,%.2f,%.2f\n\r", cnt++, pose_fb.x, pose_fb.y, pose_fb.theta, vr, vl);
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
   d_sp = sqrt(pow(dest->x, 2) + pow(dest->y, 2));            /* Distance to travel along x' */
   heading_sp = (0-pose_fb.theta) + atan2f(dest->y, dest->x); /* Required rotation to align current x with x' */

   if (fabsf(heading_sp) > (3.14f/2))
   {
      kp = 8.5f;
   }

   in_route = true;
}

bool GoToPointController::In_Route(void)
{
   return (in_route);
}

