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
extern void Update_Wheel_Speed_Setpoints(float r_sp, float l_sp);
extern void Stop(void);

#endif /* MOTOR_CONTROLS_H_ */
