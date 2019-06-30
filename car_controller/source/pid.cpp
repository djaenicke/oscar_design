/*
 * pid.cpp
 *
 *  Created on: Jun 25, 2019
 *      Author: djaenicke
 */

#include <string.h>
#include "pid.h"
#include "assert.h"

void PID::Init(PID_Cals_T * cals)
{
   if (NULL != cals)
   {
      kp = cals->k_p;
      ki = cals->k_i;
      kd = cals->k_d;
      dt = cals->dt;
   }
   else
   {
      assert(false);
   }
}

float PID::Step(float sp, float fb)
{
   float u, d; /* actuator command, derivative */
   float e = sp - fb; /* error = set point - feed back */

   d = (e - last_e)/dt;
   integral += ((e + last_e)/2)*dt; /* Trapezoidal integration */

   u = (kp * e) + (ki * integral) + (kd * d);

   last_e = e;

   return (u);
}

void PID::Reset_Integrator(void)
{
   integral = 0;
}
