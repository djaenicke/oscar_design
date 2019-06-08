/*
 * DCMotor.cpp
 *
 *  Created on: Jun 7, 2019
 *      Author: Devin
 */

#include "dc_motor.h"
#include "io_abstraction.h"
#include "assert.h"

void DC_Motor::Set_Location(Location_T new_loc)
{
   loc = new_loc;
}

void DC_Motor::Set_Direction(Direction_T new_dir)
{
   switch (new_dir)
   {
      case REVERSE:
         switch (loc)
         {
            case LEFT_SIDE:
               Set_GPIO(MOTOR_IN1, LOW);
               Set_GPIO(MOTOR_IN2, HIGH);
               break;
            case RIGHT_SIDE:
               Set_GPIO(MOTOR_IN3, HIGH);
               Set_GPIO(MOTOR_IN4, LOW);
               break;
            default:
               assert(false);
         }
         break;
      case FORWARD:
         switch (loc)
         {
            case LEFT_SIDE:
               Set_GPIO(MOTOR_IN1, HIGH);
               Set_GPIO(MOTOR_IN2, LOW);
               break;
            case RIGHT_SIDE:
               Set_GPIO(MOTOR_IN3, LOW);
               Set_GPIO(MOTOR_IN4, HIGH);
               break;
            default:
               assert(false);
         }
         break;
      default:
         assert(false);
   }
}

void DC_Motor::Set_Speed(void)
{

}

void DC_Motor::Stop(void)
{
   switch (loc)
   {
      case LEFT_SIDE:
         Set_GPIO(MOTOR_IN1, LOW);
         Set_GPIO(MOTOR_IN2, LOW);
         break;
      case RIGHT_SIDE:
         Set_GPIO(MOTOR_IN3, LOW);
         Set_GPIO(MOTOR_IN4, LOW);
         break;
      default:
         assert(false);
   }
}

void DC_Motor::Go(void)
{
   switch (loc)
   {
      case LEFT_SIDE:
         Set_GPIO(MOTOR_ENA, HIGH);
         break;
      case RIGHT_SIDE:
         Set_GPIO(MOTOR_ENB, HIGH);
         break;
      default:
         assert(false);
   }
}

