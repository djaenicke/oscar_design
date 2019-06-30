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

extern void Init_Wheel_Speed_Sensors(void);
extern void Get_Wheel_Speeds(Wheel_Speeds_T * speeds);
extern void Zero_Wheel_Speeds(void);
extern void Zero_Left_Wheel_Speeds(void);
extern void Zero_Right_Wheel_Speeds(void);

#endif /* WHEEL_SPEEDS_H_ */
