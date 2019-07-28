/*
 * inertial_states.h
 *
 *  Created on: Jul 7, 2019
 *      Author: Devin
 */

#ifndef INERTIAL_STATES_H_
#define INERTIAL_STATES_H_

#include "wheel_speeds.h"

extern void Init_Inertial_Sensors(void);
extern void Update_Robot_States(void);
extern void Get_Wheel_Ang_Velocities(Wheel_Speeds_T * ang_velocities);

#endif /* INERTIAL_STATES_H_ */
