/*
 * behaviors.h
 *
 *  Created on: Jul 28, 2019
 *      Author: Devin
 */

#ifndef BEHAVIORS_H_
#define BEHAVIORS_H_

#define CYCLE_TIME 0.025f

extern void Init_Behaviors(void);
extern void Behaviors_Task(void *pvParameters);
extern void Toggle_Autonomous_Mode(void);

#endif /* BEHAVIORS_H_ */
