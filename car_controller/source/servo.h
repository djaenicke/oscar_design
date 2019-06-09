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
public:
   void Init(void);
   void Set_Postion(uint8_t angle);
};

#endif /* SERVO_H_ */
