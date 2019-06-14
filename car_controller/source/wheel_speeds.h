/*
 * wheel_speeds.h
 *
 *  Created on: Jun 14, 2019
 *      Author: Devin
 */

#ifndef WHEEL_SPEEDS_H_
#define WHEEL_SPEEDS_H_

typedef enum
{
   RR = 0,
   RL,
   FR,
   FL,
   NUM_WHEELS
} Wheel_T;

typedef struct
{
   float rr;
   float rl;
   float fr;
   float fl;
} Wheel_Speeds_T;

void Init_Wheel_Speed_Sensors(void);
void Get_Wheel_Speeds(Wheel_Speeds_T * speeds);

#endif /* WHEEL_SPEEDS_H_ */
