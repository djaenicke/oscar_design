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
   float tol;
}PID_Cals_T;

class PID
{
private:
   float kp;
   float ki;
   float kd;
   float dt;
   float tol; /* Tolerance */
public:
   float last_e;
   float integral;
   void  Init(PID_Cals_T * cals);
   float Step(float sp, float fb, float max, float min);
   void  Reset(void);
};

#endif /* PID_H_ */
