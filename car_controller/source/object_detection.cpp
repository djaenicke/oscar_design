/*
 * object_detection.cpp
 *
 *  Created on: Jul 3, 2019
 *      Author: Devin
 */

#include "FreeRTOS.h"
#include "task.h"

#include "object_detection.h"

//#define FORWARD_ONLY

#define ROBOT_FRAME_TRANSLATION (90)
#define SENSOR_FORWARD_OFFSET   (0)  /* Degrees */
#define ROTATION_STEP           (20) /* Degrees */

typedef struct {
   float angle;
   float dist;
} Detection_T;

static Servo Sensor_Servo;
static UltrasonicSensor USS_Sensor;
static Detection_T * Detections_Ptr;
static uint8_t Max_Num_Dets;

void Init_Object_Detection(void)
{
   float min_angle, max_angle;

   Sensor_Servo.Init(SENSOR_FORWARD_OFFSET, FTM3, SERVO);

   /* Reduce the FOV to protect the servo motor */
   max_angle = Sensor_Servo.Get_Max_Angle();
   max_angle -= ROTATION_STEP;
   Sensor_Servo.Set_Max_Angle(max_angle);

   min_angle = Sensor_Servo.Get_Min_Angle();
   min_angle += ROTATION_STEP;
   Sensor_Servo.Set_Min_Angle(min_angle);

   /* Dynamically allocate an array of object detections */
   Max_Num_Dets = ((max_angle - min_angle)/ROTATION_STEP) + 1;
   Detections_Ptr = (Detection_T*) malloc(Max_Num_Dets*sizeof(Detection_T));
   assert(Detections_Ptr);
   (void)memset(Detections_Ptr, 0, Max_Num_Dets*sizeof(Detection_T));

#ifndef FORWARD_ONLY
   /* Initialize the position to max CCW */
   Sensor_Servo.Set_Angle(max_angle);
#endif

   USS_Sensor.Init(FTM2, USS_TRIGGER, USS_ECHO);
}

void Object_Detection_Task(void *pvParameters)
{
   static uint8_t det_num = 0;
   static bool first_loop = true;
   float det_dist = 0.0f;
   float cur_angle = 0.0f;

   while(1)
   {
      cur_angle = Sensor_Servo.Get_Angle();

      /* See if previous loop detected an object */
      if (!first_loop)
      {
         det_dist = USS_Sensor.Get_Obj_Dist();

         Detections_Ptr[det_num].angle = cur_angle - ROBOT_FRAME_TRANSLATION;
         Detections_Ptr[det_num].dist  = det_dist;
         det_num++;

         if (det_num == Max_Num_Dets)
         {
            det_num = 0;
         }
      }
      else
      {
         first_loop = false;
      }

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

      /* Task cycle time reqs:
       *   - cycle time > 2 * (max range / speed of sound)
       *   - cycle time > 1.67 (ms) * ROTATION_STEP
       */
      vTaskDelay(pdMS_TO_TICKS(150));
   }
}

