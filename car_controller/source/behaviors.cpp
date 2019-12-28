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
#include "udp_client.h"
#include "ip_app_iface.h"
#include "fsl_debug_console.h"

#include <ros.h>
#include <std_msgs/String.h>

#define S_2_MS 1000
#define NUM_WAYPOINTS (1)

/* Tuning parameters */
#define Kp  (4.0f)
#define TOLERANCE (0.01) /* (m) */
#define GTP_SPEED (0.5)  /* (m/s) */

#define RX_BUFFER_SIZE 100

static GoToPointController GTP_Controller;
bool auto_mode_active = false;

static Destination_T Waypoints[NUM_WAYPOINTS] = {4,0};
static uint8_t Current_Waypoint = 0;

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void Behaviors_Task(void *pvParameters)
{
   TickType_t xLastWakeTime;
   static bool nh_initialized = false;

   xLastWakeTime = xTaskGetTickCount();

   while(1)
   {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CYCLE_TIME*S_2_MS));

      if (CONNECTED == Get_Network_Status())
      {
         if (!nh_initialized)
         {
            nh.initNode();
            nh.advertise(chatter);
            nh_initialized = true;
         }
         else
         {
            str_msg.data = hello;
            chatter.publish(&str_msg);
            nh.spinOnce();
         }
      }

      Update_Robot_States();
      Run_Object_Detection();

      if (auto_mode_active)
      {
         if (GTP_Controller.In_Route())
         {
            GTP_Controller.Execute();
         }
         else
         {
            if (Current_Waypoint < NUM_WAYPOINTS)
            {
               GTP_Controller.Update_Destination(&(Waypoints[Current_Waypoint]));
               Current_Waypoint++;
            }
            else
            {
               Toggle_Autonomous_Mode();
            }
         }
      }

      Run_Motor_Controls();
   }
}

void Init_Behaviors(void)
{
   GTP_Controller.Init(TOLERANCE, Kp, GTP_SPEED);
}

void Toggle_Autonomous_Mode(void)
{
   if (auto_mode_active)
   {
      auto_mode_active = false;
   }
   else
   {
      auto_mode_active = true;
   }

   Stop();
}
