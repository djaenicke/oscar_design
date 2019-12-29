/*
 * inertial_states.cpp
 *
 *  Created on: Jul 7, 2019
 *      Author: Devin
 */

#include <math.h>
#include <string.h>
#include "assert.h"
#include "behaviors.h"
#include "inertial_states.h"
#include "constants.h"
#include "wheel_speeds.h"

static Pose_T Pose = {0};

static float X_Dot = 0;
static float X_Dot_Last = 0;

static float Y_Dot = 0;
static float Y_Dot_Last = 0;

static float Theta_Dot = 0;
static float Theta_Dot_Last = 0;

void Update_Robot_States(void)
{
   float vr, vl, v = 0;
   Wheel_Speeds_T wheel_ang_v;

   Get_Wheel_Ang_Velocities(&wheel_ang_v);

   /* Compute the robot's linear wheel velocities */
   vr = wheel_ang_v.r * WHEEL_RADIUS;
   vl = wheel_ang_v.l * WHEEL_RADIUS;

   /* Compute the robot's linear velocity based on the wheel speeds */
   v = ((vr + vl)/2);

   /* Compute the robot's angular velocity */
   Theta_Dot = ((vr - vl)/WHEEL_BASE);

   /* Perform trapezoidal integration on the angular velocity to get the heading angle */
   Pose.theta += (((Theta_Dot + Theta_Dot_Last)/2)*CYCLE_TIME);

   /* Compute the robot's velocity components */
   X_Dot = v*cosf(Pose.theta);
   Y_Dot = v*sinf(Pose.theta);

   /* Perform trapezoidal integration on the velocity components to get the X and Y positions */
   Pose.x += (((X_Dot + X_Dot_Last)/2)*CYCLE_TIME);
   Pose.y += (((Y_Dot + Y_Dot_Last)/2)*CYCLE_TIME);

   /* Store the current values for the next iteration */
   X_Dot_Last = X_Dot;
   Y_Dot_Last = Y_Dot;
   Theta_Dot_Last = Theta_Dot;
}

void Get_Pose(Pose_T * dest)
{
   assert(dest);
   (void) memcpy(dest, &Pose, sizeof(Pose_T));
}

void Reset_Inertial_Data(void)
{
   Pose.x = 0;
   Pose.y = 0;
   Pose.theta = 0;

   X_Dot = 0;
   X_Dot_Last = 0;

   Y_Dot = 0;
   Y_Dot_Last = 0;

   Theta_Dot = 0;
   Theta_Dot_Last = 0;
}

