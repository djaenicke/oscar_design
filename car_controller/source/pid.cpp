/*
 * pid.cpp
 *
 *  Created on: Jun 25, 2019
 *      Author: djaenicke
 */

#include <stdint.h>
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
   float u, d;          /* actuator command, derivative  */
   float ti = integral; /* temporary integral            */
   float e = sp - fb;   /* error = set point - feed back */
   int8_t sign;

   /* Ignore the error if it's smaller than the tolerance */
   if (fabs(e) < tol)
   {
      e = 0.0f;
   }

   d = (e - last_e)/dt;
   ti += (((e + last_e)/2)*dt); /* Trapezoidal integration */

   u = (kp * e) + (ki * integral) + (kd * d);

   last_e = e;

   /* Determine the sign of u */
   sign = signbit(u) ? -1 : 1;

   /* Limit the integral term */
   if (fabs(u) < max)
   {
      integral = ti;
   }

   /* Perform saturation on the activation signal */
   if (fabs(u) < min)
   {
      u = sign * min;
   }
   else if (fabs(u) > max)
   {
      u = sign * max;
   }

   return (u);
}

void PID::Reset(void)
{
   last_e = 0.0f;
   integral = 0.0f;
}

