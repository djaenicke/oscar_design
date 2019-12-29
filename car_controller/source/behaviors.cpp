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
#include "ip_app_iface.h"
#include "fsl_debug_console.h"
#include "mpu6050.h"
#include "FXOS8700CQ.h"
#include "wheel_speeds.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>

#define S_2_MS 1000
#define NUM_WAYPOINTS (1)

/* Tuning parameters */
#define Kp  (4.0f)
#define TOLERANCE (0.01) /* (m) */
#define GTP_SPEED (0.5)  /* (m/s) */

static GoToPointController GTP_Controller;
static Destination_T Waypoints[NUM_WAYPOINTS] = {4,0};
static uint8_t Current_Waypoint = 0;
static bool Auto_Mode_Active = false;

/* IMU objects */
static MPU6050    My_MPU6050;
static FXOS8700CQ My_FXOS8700CQ;

/* ROS Client Data */
static ros::NodeHandle nh;
static sensor_msgs::Imu IMU_Msgs[2];

void Behaviors_Task(void *pvParameters)
{
   TickType_t xLastWakeTime;
   static bool nh_initialized = false;

   Wheel_Speeds_T wheel_ang_v;
   Accel_Data_T accel_data = {0};
   Gyro_Data_T  gyro_data  = {0};

   xLastWakeTime = xTaskGetTickCount();

   while(1)
   {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CYCLE_TIME*S_2_MS));

      if (CONNECTED == Get_Network_Status())
      {
         if (!nh_initialized)
         {
            nh.initNode();
            nh_initialized = true;
         }
         else
         {
            nh.spinOnce();
         }
      }

      Measure_Wheel_Speeds();
      Get_Wheel_Ang_Velocities(&wheel_ang_v);
      //My_MPU6050.Read_Accel_Data(&accel_data);
      //My_MPU6050.Read_Gyro_Data(&gyro_data);

      Run_Object_Detection();

      if (Auto_Mode_Active)
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
   /* Initialize the external 6-axis MPU6050 */
   My_MPU6050.Init(FTM0, I2C1);

   /* Initialize the 6-axis on board FXOS8700CQ */
   My_FXOS8700CQ.Init();

   /* Initialize the Go To Point controller */
   GTP_Controller.Init(TOLERANCE, Kp, GTP_SPEED);
}

void Toggle_Autonomous_Mode(void)
{
   if (Auto_Mode_Active)
   {
      Auto_Mode_Active = false;
   }
   else
   {
      Auto_Mode_Active = true;
   }

   Stop();
}
