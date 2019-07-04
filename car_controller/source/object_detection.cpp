/*
 * object_detection.cpp
 *
 *  Created on: Jul 3, 2019
 *      Author: Devin
 */

#include "FreeRTOS.h"
#include "task.h"

#include "servo.h"

#define SENSOR_FORWARD_OFFSET (0)  /* Degrees */
#define ROTATION_STEP         (20) /* Degrees */

static Servo Sensor_Servo;

void Init_Object_Detection(void)
{
   Sensor_Servo.Init(SENSOR_FORWARD_OFFSET);

   /* Reduce the FOV to protect the servo motor */
   Sensor_Servo.max_angle -= ROTATION_STEP;
   Sensor_Servo.min_angle += ROTATION_STEP;

   Sensor_Servo.Set_Postion(Sensor_Servo.max_angle);
}

void Object_Detection_Task(void *pvParameters)
{
   while(1)
   {
      if ((Sensor_Servo.cur_angle - ROTATION_STEP) >= Sensor_Servo.min_angle)
      {
         Sensor_Servo.Set_Postion(Sensor_Servo.cur_angle - ROTATION_STEP);
      }
      else
      {
         Sensor_Servo.Set_Postion(Sensor_Servo.max_angle);
      }

      /* Scan for object here */

      /* Delay should be at least 2 * (range / speed of sound) */
      vTaskDelay(pdMS_TO_TICKS(200));
   }
}

