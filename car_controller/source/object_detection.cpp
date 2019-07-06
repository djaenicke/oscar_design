/*
 * object_detection.cpp
 *
 *  Created on: Jul 3, 2019
 *      Author: Devin
 */

#include "FreeRTOS.h"
#include "task.h"

#include "object_detection.h"

#define FORWARD_ONLY

#define SENSOR_FORWARD_OFFSET (0)  /* Degrees */
#define ROTATION_STEP         (20) /* Degrees */

static Servo Sensor_Servo;
static UltrasonicSensor USS_Sensor;

void Init_Object_Detection(void)
{
   float temp_angle;

   Sensor_Servo.Init(SENSOR_FORWARD_OFFSET, FTM3, SERVO);

   /* Reduce the FOV to protect the servo motor */
   temp_angle = Sensor_Servo.Get_Max_Angle();
   temp_angle -= ROTATION_STEP;
   Sensor_Servo.Set_Max_Angle(temp_angle);

   temp_angle = Sensor_Servo.Get_Min_Angle();
   temp_angle += ROTATION_STEP;
   Sensor_Servo.Set_Min_Angle(temp_angle);

#ifndef FORWARD_ONLY
   /* Initialize the position to max CCW */
   temp_angle = Sensor_Servo.Get_Max_Angle();
   Sensor_Servo.Set_Angle(temp_angle);
#endif

   USS_Sensor.Init(FTM2, USS_TRIGGER, USS_ECHO);
}

void Object_Detection_Task(void *pvParameters)
{
   static bool first_loop = true;
   float obj_dist = 0.0f;
   float cur_angle = 0.0f;

   while(1)
   {
      /* See if previous loop detected an object */
      if (!first_loop)
      {
         obj_dist = USS_Sensor.Get_Obj_Dist();
      }
      else
      {
         first_loop = false;
      }

      cur_angle = Sensor_Servo.Get_Angle();

#ifndef FORWARD_ONLY
      if ((cur_angle - ROTATION_STEP) >= Sensor_Servo.Get_Min_Angle())
      {
         Sensor_Servo.Set_Angle(cur_angle - ROTATION_STEP);
      }
      else
      {
         cur_angle = Sensor_Servo.Get_Max_Angle();
         Sensor_Servo.Set_Angle(cur_angle);
      }
#endif

      /* Scan for object here */
      USS_Sensor.Trigger();

      /* Delay should be at least 2 * (range / speed of sound) */
      vTaskDelay(pdMS_TO_TICKS(200));
   }
}

