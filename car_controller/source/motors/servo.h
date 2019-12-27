/*
 * servo.h
 *
 *  Created on: Jun 8, 2019
 *      Author: Devin
 */

#ifndef SERVO_H_
#define SERVO_H_

#include "fsl_ftm.h"
#include "io_abstraction.h"

#define MIN_ANGLE_DEG (0.0f)
#define DEF_ANGLE_DEG (90.0f)
#define MAX_ANGLE_DEG (180.0f)

typedef enum {
   DC_OFF,
   DC_ON
} PWM_DC_State_T;

typedef struct {
   FTM_Type * ftm_ptr;
   PWM_DC_State_T dc_state;
   uint16_t on_time;
   uint16_t off_time;
   uint16_t period;
   IO_Map_T pin;
} Software_PWM_T;

class Servo
{
private:
   Software_PWM_T pwm;
   bool  init_complete = false;
   float position_offset = 0;
   float cur_angle = DEF_ANGLE_DEG;
   float min_angle = MIN_ANGLE_DEG;
   float max_angle = MAX_ANGLE_DEG;
public:
   void  Init(float offset, FTM_Type *ftm_base_ptr, IO_Map_T servo_pin);

   float Get_Angle(void);
   float Get_Max_Angle(void);
   float Get_Min_Angle(void);

   void Set_Angle(float angle);
   void Set_Max_Angle(float angle);
   void Set_Min_Angle(float angle);
};

#endif /* SERVO_H_ */
