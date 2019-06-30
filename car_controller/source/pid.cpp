/*
 * pid.cpp
 *
 *  Created on: Jun 25, 2019
 *      Author: djaenicke
 */

#include <string.h>
#include <math.h>
#include "pid.h"
#include "assert.h"

void PID::Init(PID_Cals_T * cals)
{
   if (NULL != cals)
   {
      kp  = cals->k_p;
      ki  = cals->k_i;
      kd  = cals->k_d;
      dt  = cals->dt;
      tol = cals->tol;
   }
   else
   {
      assert(false);
   }
}

float PID::Step(float sp, float fb, float max, float min)
{
   float u, d; /* actuator command, derivative */
   float e = sp - fb; /* error = set point - feed back */

   /* Ignore the error if it's smaller than the tolerance */
   if (fabs(e) < tol)
   {
      e = 0.0f;
   }

   d = (e - last_e)/dt;
   integral += ((e + last_e)/2)*dt; /* Trapezoidal integration */

   u = (kp * e) + (ki * integral) + (kd * d);

   last_e = e;

   /* Perform saturation to prevent integral wind up */
   if (fabs(u) > max)
   {
      if (signbit(u))
      {
         /* Less than max negative */
         integral += abs(u) - max;
         u = -max;
      }
      else
      {
         /* Greater than max positive */
         integral -= u - max;
         u = max;
      }
   }
   else if (fabs(u) < min)
   {
      if (signbit(u))
      {
         /* Less than min and negative */
         integral += abs(u) - min;
         u = -min;
      }
      else
      {
         /* Less than min but not negative */
         integral += min - u;
         u = min;
      }
   }

   return (u);
}

void PID::Reset(void)
{
   last_e = 0.0f;
   integral = 0.0f;
}
