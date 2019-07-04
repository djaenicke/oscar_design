/*
 * object_detection.cpp
 *
 *  Created on: Jul 3, 2019
 *      Author: Devin
 */

#include "FreeRTOS.h"
#include "task.h"

#include "object_detection.h"

#define SENSOR_FORWARD_OFFSET (0)  /* Degrees */
#define ROTATION_STEP         (20) /* Degrees */

static Servo Sensor_Servo;
static UltrasonicSensor USS_Sensor;

void Init_Object_Detection(void)
{
   float temp_angle;

   Sensor_Servo.Init(SENSOR_FORWARD_OFFSET, FTM3);

   /* Reduce the FOV to protect the servo motor */
   temp_angle = Sensor_Servo.Get_Max_Angle();
   temp_angle -= ROTATION_STEP;
   Sensor_Servo.Set_Max_Angle(temp_angle);

   temp_angle = Sensor_Servo.Get_Min_Angle();
   temp_angle += ROTATION_STEP;
   Sensor_Servo.Set_Min_Angle(temp_angle);

   /* Initialize the position to max CCW */
   temp_angle = Sensor_Servo.Get_Max_Angle();
   Sensor_Servo.Set_Angle(temp_angle);

   USS_Sensor.Init(FTM2, USS_TRIGGER, USS_ECHO);
}

void Object_Detection_Task(void *pvParameters)
{
   float cur_angle;

   while(1)
   {
      cur_angle = Sensor_Servo.Get_Angle();

      if ((cur_angle - ROTATION_STEP) >= Sensor_Servo.Get_Min_Angle())
      {
         Sensor_Servo.Set_Angle(cur_angle - ROTATION_STEP);
      }
      else
      {
         cur_angle = Sensor_Servo.Get_Max_Angle();
         Sensor_Servo.Set_Angle(cur_angle);
      }

      /* Scan for object here */
      USS_Sensor.Trigger();

      /* Delay should be at least 2 * (range / speed of sound) */
      vTaskDelay(pdMS_TO_TICKS(200));
   }
}

