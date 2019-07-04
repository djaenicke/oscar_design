/*
 * object_detection.cpp
 *
 *  Created on: Jul 3, 2019
 *      Author: Devin
 */

#include "servo.h"

#define SENSOR_FORWARD_OFFSET (0) /* Degrees */

static Servo Sensor_Servo;

void Init_Object_Detection(void)
{
   Sensor_Servo.Init(SENSOR_FORWARD_OFFSET);
}


