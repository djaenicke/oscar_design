/*
 * motor_controls.h
 *
 *  Created on: Jun 7, 2019
 *      Author: Devin
 */

#ifndef MOTOR_CONTROLS_H_
#define MOTOR_CONTROLS_H_

extern void Init_Motor_Controls(void);
extern void Motor_Controls_Task(void *pvParameters);
extern void Forward(void);
extern void Backward(void);
extern void Left(void);
extern void Right(void);
extern void Stop(void);

#endif /* MOTOR_CONTROLS_H_ */
