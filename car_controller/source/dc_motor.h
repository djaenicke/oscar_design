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
public:
   ftm_chnl_t pwm_channel;

   void Set_Location(Location_T loc);
   void Set_Direction(Direction_T dir);
   void Set_Speed(uint8_t percent); /* TODO - replace with rad/s */
   void Stop(void);
};

#endif /* DC_MOTOR_H_ */
