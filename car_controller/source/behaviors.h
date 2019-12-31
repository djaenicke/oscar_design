/*
 * behaviors.h
 *
 *  Created on: Jul 28, 2019
 *      Author: Devin
 */

#ifndef BEHAVIORS_H_
#define BEHAVIORS_H_

#define CYCLE_TIME 0.025f
#define WHEEL_BASE   (0.134f)  /* (m) */
#define WHEEL_RADIUS (0.0336f) /* (m) */

typedef struct {
   float x;
   float y;
   float theta;
} Pose_T;

extern void Init_Behaviors(void);
extern void Behaviors_Task(void *pvParameters);
extern void Toggle_Autonomous_Mode(void);
extern void Get_Pose(Pose_T * dest);

#endif /* BEHAVIORS_H_ */
