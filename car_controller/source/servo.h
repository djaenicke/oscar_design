/*
 * servo.h
 *
 *  Created on: Jun 8, 2019
 *      Author: Devin
 */

#ifndef SERVO_H_
#define SERVO_H_

#include "fsl_ftm.h"

class Servo
{
private:
   bool init_complete;
   float position_offset = 0;
public:
   float min_angle;
   float max_angle;

   void Init(float offset);
   void Set_Postion(float angle);
};

#endif /* SERVO_H_ */
