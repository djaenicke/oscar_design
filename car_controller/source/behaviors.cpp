/*
 * behaviors.cpp
 *
 *  Created on: Jul 28, 2019
 *      Author: Devin
 */


#include "FreeRTOS.h"
#include "task.h"

#include "behaviors.h"
#include "inertial_states.h"
#include "object_detection.h"
#include "go_to_point.h"
#include "motor_controls.h"

#define S_2_MS 1000

void Behaviors_Task(void *pvParameters)
{
   while(1)
   {
      Update_Robot_States();

      Run_Object_Detection();
      Run_Go_To_Point_Controller();
      Run_Motor_Controls();

      vTaskDelay(pdMS_TO_TICKS(CYCLE_TIME*S_2_MS));
   }
}
