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
   R_E = 0,
   L_E,
   R_HE,
   L_HE,
   NUM_WHEEL_SENSORS
} Wheel_Sensor_T;

typedef struct
{
   float r;
   float l;
   float r_he;
   float l_he;
} Wheel_Speeds_T;

extern void Init_Wheel_Speed_Sensors(void);
extern void Get_Wheel_Speeds(Wheel_Speeds_T * speeds);
extern void Zero_Wheel_Speed(Wheel_Sensor_T sensor);

#endif /* WHEEL_SPEEDS_H_ */
