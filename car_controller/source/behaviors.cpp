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

#define S_2_MS 1000
#define NUM_WAYPOINTS (1)

/* Tuning parameters */
#define Kp  (4.0f)
#define TOLERANCE (0.01) /* (m) */
#define GTP_SPEED (0.5)  /* (m/s) */

#define RX_BUFFER_SIZE 100

static UdpClient ROS_UDP;
static char rx_buffer[RX_BUFFER_SIZE];

static GoToPointController GTP_Controller;
bool auto_mode_active = false;

static Destination_T Waypoints[NUM_WAYPOINTS] = {4,0};
static uint8_t Current_Waypoint = 0;

void Behaviors_Task(void *pvParameters)
{
   uint16_t i = 0;

   while(1)
   {
      if (CONNECTED == Get_Network_Status())
      {
         if (!ROS_UDP.Is_Initialized())
         {
            ROS_UDP.Init(Get_Netif(), 5000);
         }

         if (ROS_UDP.Is_Initialized() && !ROS_UDP.Is_Connected())
         {
            ROS_UDP.Set_Remote_Ip("192.168.1.4");
            ROS_UDP.Set_Remote_Port(5000);
            ROS_UDP.Connect();
         }

         if (ROS_UDP.Is_Connected())
         {
            ROS_UDP.Send_Datagram("Sent from ROS embedded client!", strlen("Sent from ROS embedded client!"));
            if (ROS_UDP.Rx_Bytes_Available())
            {
               ROS_UDP.Read_Datagram(rx_buffer, RX_BUFFER_SIZE);
               PRINTF("%s", rx_buffer);
            }
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

      vTaskDelay(pdMS_TO_TICKS(CYCLE_TIME*S_2_MS));
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
