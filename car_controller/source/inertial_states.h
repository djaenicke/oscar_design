/*
 * inertial_states.h
 *
 *  Created on: Jul 7, 2019
 *      Author: Devin
 */

#ifndef INERTIAL_STATES_H_
#define INERTIAL_STATES_H_

#include "wheel_speeds.h"

#define WHEEL_BASE   (0.145f)  /* (m) */
#define WHEEL_RADIUS (0.0336f) /* (m) */

typedef struct {
   float x;
   float y;
   float theta;
} Pose_T;

extern void Init_Inertial_Sensors(void);
extern void Update_Robot_States(void);
extern void Get_Wheel_Ang_Velocities(Wheel_Speeds_T * ang_velocities);
extern void Get_Pose(Pose_T * dest);

#endif /* INERTIAL_STATES_H_ */
