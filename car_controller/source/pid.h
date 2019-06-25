/*
 * PID.h
 *
 *  Created on: Jun 25, 2019
 *      Author: djaenicke
 */

#ifndef PID_H_
#define PID_H_

typedef struct
{
   float k_p;
   float k_i;
   float k_d;
   float dt;
}PID_Cals_T;

class PID
{
private:
   float kp;
   float ki;
   float kd;
   float dt;
   float last_e;
   float integral;
public:
   void  Init(PID_Cals_T * cals);
   float Step(float sp, float fb);
   void  Reset_Integrator(void);
};

#endif /* PID_H_ */
