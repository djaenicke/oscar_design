/*
 * DCMotor.h
 *
 *  Created on: Jun 7, 2019
 *      Author: Devin
 */

#ifndef DC_MOTOR_H_
#define DC_MOTOR_H_

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
   void Set_Location(Location_T loc);
   void Set_Direction(Direction_T dir);
   void Set_Speed(void);
   void Stop(void);
   void Go(void);
};

#endif /* DC_MOTOR_H_ */
