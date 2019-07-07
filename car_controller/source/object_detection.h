/*
 * object_detection.h
 *
 *  Created on: Jul 3, 2019
 *      Author: Devin
 */

#ifndef OBJECT_DETECTION_H_
#define OBJECT_DETECTION_H_

#include "servo.h"
#include "ultrasonic_sensor.h"

extern void Init_Object_Detection(void);
extern void Object_Detection_Task(void *pvParameters);
extern void Toggle_Obj_Det_Enable(void);

#endif /* OBJECT_DETECTION_H_ */
