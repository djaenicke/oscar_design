/*
 * behaviors.cpp
 *
 *  Created on: Jul 28, 2019
 *      Author: Devin
 */


#include "FreeRTOS.h"
#include "task.h"

#include "behaviors.h"
#include "app_supervisor.h"
#include "object_detection.h"
#include "go_to_point.h"
#include "motor_controls.h"
#include "ip_app_iface.h"
#include "fsl_debug_console.h"
#include "mpu6050.h"
#include "FXOS8700CQ.h"
#include "wheel_speeds.h"
#include "delay.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#define S_2_MS 1000
#define NUM_WAYPOINTS (1)

/* Tuning parameters */
#define Kp  (4.0f)
#define TOLERANCE (0.01) /* (m) */
#define GTP_SPEED (0.5)  /* (m/s) */

#define _MPU6050    1
#define _FXOS8700CQ 2

static GoToPointController GTP_Controller;
static Destination_T Waypoints[NUM_WAYPOINTS] = {4,0};
static uint8_t Current_Waypoint = 0;
static bool Auto_Mode_Active = false;

/* IMU objects */
static MPU6050    My_MPU6050;
static FXOS8700CQ My_FXOS8700CQ;

/* ROS Client Data */
static ros::NodeHandle nh;

static sensor_msgs::Imu IMU_Msg;
static nav_msgs::Odometry Odo_Msg;

ros::Publisher imu("imu_data", &IMU_Msg);
ros::Publisher odo("odo_data", &Odo_Msg);

static void Behaviors_Task(void *pvParameters);

void Behaviors_Task(void *pvParameters)
{
   TickType_t xLastWakeTime;
   static bool nh_initialized = false;

   Wheel_Speeds_T wheel_ang_v;
   Accel_Data_T accel_data = {0};
   Gyro_Data_T  gyro_data  = {0};
   float vr, vl = 0;

   xLastWakeTime = xTaskGetTickCount();

   while(1)
   {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CYCLE_TIME*S_2_MS));

      Measure_Wheel_Speeds();
      Get_Wheel_Ang_Velocities(&wheel_ang_v);

      /* Compute the robot's linear wheel velocities */
      vr = wheel_ang_v.r * WHEEL_RADIUS;
      vl = wheel_ang_v.l * WHEEL_RADIUS;

      Odo_Msg.header.stamp = nh.now();

      /* Compute the robot's linear velocity based on the wheel speeds */
      Odo_Msg.twist.twist.linear.x = ((vr + vl)/2);

      /* Compute the robot's angular velocity */
      Odo_Msg.twist.twist.angular.z = ((vr - vl)/WHEEL_BASE);

      My_MPU6050.Read_Accel_Data(&accel_data);
      My_MPU6050.Read_Gyro_Data(&gyro_data);

      IMU_Msg.header.stamp = nh.now();
      IMU_Msg.linear_acceleration.x = accel_data.ax;
      IMU_Msg.linear_acceleration.y = accel_data.ay;
      IMU_Msg.linear_acceleration.z = accel_data.az;

      IMU_Msg.angular_velocity.x = gyro_data.gx;
      IMU_Msg.angular_velocity.y = gyro_data.gy;
      IMU_Msg.angular_velocity.z = gyro_data.gz;

      if (CONNECTED == Get_Network_Status())
      {
         if (!nh_initialized)
         {
            nh.initNode();
            nh.advertise(imu);
            nh.advertise(odo);
            nh_initialized = true;
         }
         else
         {
            imu.publish(&IMU_Msg);
            odo.publish(&Odo_Msg);
            nh.spinOnce();
         }
      }

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

void Init_Behaviors_Task(void *pvParameters)
{
   /* Initialize the external 6-axis MPU6050 */
   My_MPU6050.Init();

   /* Initialize the 6-axis on board FXOS8700CQ */
   My_FXOS8700CQ.Init();

   /* Initialize the Go To Point controller */
   GTP_Controller.Init(TOLERANCE, Kp, GTP_SPEED);

   /* Configure the different frame ids */
   IMU_Msg.header.frame_id = "base_link";
   Odo_Msg.header.frame_id = "base_link";

   if (pdPASS != xTaskCreate(Behaviors_Task, "Behaviors_Task", 4096, NULL, BEHAVIORS_TASK_PRIO, NULL))
   {
      assert(0);
   }

   vTaskSuspend(NULL);
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

void Get_Pose(Pose_T * dest)
{
   assert(dest);

   dest->x = (float) Odo_Msg.pose.pose.position.x;
   dest->y = (float) Odo_Msg.pose.pose.position.y;
   dest->theta = (float) Odo_Msg.pose.pose.orientation.w;
}
