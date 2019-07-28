/*
 * go_to_point.cpp
 *
 *  Created on: Jul 28, 2019
 *      Author: Devin
 */

#include <math.h>

#include "go_to_point.h"
#include "inertial_states.h"
#include "motor_controls.h"

/* Tuning parameters */
#define Kp (4.0f)
#define X_TOL (0.05) /* (m) */
#define Y_TOL (0.05) /* (m) */

static Pose_T Pose_SP = {0};
static float Robot_V = 0;

/* Controller Design */
/* https://pdfs.semanticscholar.org/edde/fa921e26efbbfd6c65ad1e13af0bbbc1b946.pdf */

void Run_Go_To_Point_Controller(void)
{
   float vr, vl; /* Desired linear wheel velocities */
   float theta_d, theta_e; /* Heading difference and heading error */
   float omega;
   Pose_T pose_fb; /* Pose feedback */

   /* Get the heading and X and Y positions */
   Get_Pose(&pose_fb);

   if ((fabsf(Pose_SP.x - pose_fb.x) > X_TOL) && (fabsf(Pose_SP.y - pose_fb.y) > Y_TOL))
   {
      theta_d = Pose_SP.theta - pose_fb.theta; /* Heading difference */
      theta_e = atan2f(sinf(theta_d), cosf(theta_d)); /* Heading error */

      /* Proportional control */
      omega = Kp * theta_e;

      /* Determine the required linear velocities */
      vr = Robot_V + (omega*WHEEL_BASE/2);
      vl = Robot_V - (omega*WHEEL_BASE/2);

      Update_Wheel_Angular_V_SP(vl/WHEEL_RADIUS, vr/WHEEL_RADIUS, false);
   }
   else
   {
      /* Within X_TOL and Y_TOL of destination */
      Stop();
   }
}

void Update_Destination(float x, float y, float robot_v)
{
   /* Compute the go-to-point controller set point */
   Pose_SP.x = x;
   Pose_SP.y = y;
   Pose_SP.theta = atan2f(y, x);

   /* Linear velocity in (m/s) */
   Robot_V = robot_v;
}

