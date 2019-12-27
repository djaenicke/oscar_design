/*
 * DCMotor.h
 *
 *  Created on: Jun 7, 2019
 *      Author: Devin
 */

#ifndef DC_MOTOR_H_
#define DC_MOTOR_H_

#include "fsl_ftm.h"

typedef enum
{
    UNKNOWN_DIR = 0,
    FORWARD,
    REVERSE
} Direction_T;

typedef enum
{
    UNKNOWN_LOC = 0,
    RIGHT_SIDE,
    LEFT_SIDE
} Location_T;

class DC_Motor
{
private:
   Location_T loc = UNKNOWN_LOC;
   Direction_T direction = UNKNOWN_DIR;
public:
   ftm_chnl_t pwm_channel;
   bool stopped = true;

   void Set_Location(Location_T loc);
   void Set_Direction(Direction_T dir);
   Direction_T Get_Direction(void);
   void Set_DC(uint8_t percent);
   void Freewheel(void);
   void Stop(void);
};

#endif /* DC_MOTOR_H_ */
