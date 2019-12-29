/*
 * inertial_states.h
 *
 *  Created on: Jul 7, 2019
 *      Author: Devin
 */

#ifndef INERTIAL_STATES_H_
#define INERTIAL_STATES_H_

#define WHEEL_BASE   (0.134f)  /* (m) */
#define WHEEL_RADIUS (0.0336f) /* (m) */

typedef struct {
   float x;
   float y;
   float theta;
} Pose_T;

extern void Update_Robot_States(void);
extern void Get_Pose(Pose_T * dest);
extern void Reset_Inertial_Data(void);

#endif /* INERTIAL_STATES_H_ */
