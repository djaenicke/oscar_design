/*
 * motor_controls.h
 *
 *  Created on: Jun 7, 2019
 *      Author: Devin
 */

#ifndef MOTOR_CONTROLS_H_
#define MOTOR_CONTROLS_H_

#include <stdint.h>

extern void Init_Motor_Controls(void);
extern void Run_Motor_Controls(void);
extern void Update_Wheel_Angular_V_SP(float r_sp, float l_sp, bool reset_pid);
extern bool Right_Motor_Stopped(void);
extern bool Left_Motor_Stopped(void);
extern void Stop(void);
extern int8_t Right_Wheel_Speed_Sign(void);
extern int8_t Left_Wheel_Speed_Sign(void);

#endif /* MOTOR_CONTROLS_H_ */
