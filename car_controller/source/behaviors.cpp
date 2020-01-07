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

static sensor_msgs::Imu IMU_Msg_MPU;
static sensor_msgs::Imu IMU_Msg_FXOS;
static nav_msgs::Odometry Odo_Msg;

ros::Publisher imu_mpu("imu_data_mpu", &IMU_Msg_MPU);
ros::Publisher imu_fxos("imu_data_fxos", &IMU_Msg_FXOS);
ros::Publisher odo("odo_data", &Odo_Msg);

static inline void Populate_IMU_Covariances(void);
static void Behaviors_Task(void *pvParameters);
static void Populate_Odo_Msg(void);
static void Populate_IMU_Msgs(void);
static void Publish_ROS_Topics(void);
static inline void Run_GTP_Controller(void);

void Behaviors_Task(void *pvParameters)
{
   TickType_t xLastWakeTime;

   xLastWakeTime = xTaskGetTickCount();

   while(1)
   {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CYCLE_TIME*S_2_MS));

      Populate_Odo_Msg();
      Populate_IMU_Msgs();
      Publish_ROS_Topics();
      Run_Object_Detection();
      Run_GTP_Controller();
      Run_Motor_Controls();
   }
}

void Init_Behaviors_Task(void *pvParameters)
{
   /* Initialize the external 6-axis MPU6050 */
   My_MPU6050.Init(AFS_2G, GFS_250DPS);

   /* Initialize the 6-axis on board FXOS8700CQ */
   My_FXOS8700CQ.Init(FXOS_2G);

   /* Initialize the Go To Point controller */
   GTP_Controller.Init(TOLERANCE, Kp, GTP_SPEED);

   /* Configure the different frame ids */
   IMU_Msg_MPU.header.frame_id = "odom";
   IMU_Msg_FXOS.header.frame_id = "odom";
   Odo_Msg.header.frame_id = "odom";

   Populate_IMU_Covariances();

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

static inline void Populate_Covariances(void)
{
   /* Row major about x, y, z axes */
   IMU_Msg_MPU.orientation_covariance[0] = -1; /* No orientation measurement available */

   /* Row major about x, y, z axes */
   IMU_Msg_MPU.angular_velocity_covariance[0] = 0;
   IMU_Msg_MPU.angular_velocity_covariance[1] = 0;
   IMU_Msg_MPU.angular_velocity_covariance[2] = 0;
   IMU_Msg_MPU.angular_velocity_covariance[3] = 0;
   IMU_Msg_MPU.angular_velocity_covariance[4] = 0;
   IMU_Msg_MPU.angular_velocity_covariance[5] = 0;
   IMU_Msg_MPU.angular_velocity_covariance[6] = 0;
   IMU_Msg_MPU.angular_velocity_covariance[7] = 0;
   IMU_Msg_MPU.angular_velocity_covariance[8] = 0;

   /* Row major x, y z */
   IMU_Msg_MPU.linear_acceleration_covariance[0] = 0;
   IMU_Msg_MPU.linear_acceleration_covariance[1] = 0;
   IMU_Msg_MPU.linear_acceleration_covariance[2] = 0;
   IMU_Msg_MPU.linear_acceleration_covariance[3] = 0;
   IMU_Msg_MPU.linear_acceleration_covariance[4] = 0;
   IMU_Msg_MPU.linear_acceleration_covariance[5] = 0;
   IMU_Msg_MPU.linear_acceleration_covariance[6] = 0;
   IMU_Msg_MPU.linear_acceleration_covariance[7] = 0;
   IMU_Msg_MPU.linear_acceleration_covariance[8] = 0;

   /* Row major about x, y, z axes */
   IMU_Msg_FXOS.orientation_covariance[0] = -1; /* No orientation measurement available (yet) */

   /* Row major about x, y, z axes */
   IMU_Msg_FXOS.angular_velocity_covariance[0] = -1; /* No orientation measurement available */

   /* Row major x, y z */
   IMU_Msg_FXOS.linear_acceleration_covariance[0] = 0;
   IMU_Msg_FXOS.linear_acceleration_covariance[1] = 0;
   IMU_Msg_FXOS.linear_acceleration_covariance[2] = 0;
   IMU_Msg_FXOS.linear_acceleration_covariance[3] = 0;
   IMU_Msg_FXOS.linear_acceleration_covariance[4] = 0;
   IMU_Msg_FXOS.linear_acceleration_covariance[5] = 0;
   IMU_Msg_FXOS.linear_acceleration_covariance[6] = 0;
   IMU_Msg_FXOS.linear_acceleration_covariance[7] = 0;
   IMU_Msg_FXOS.linear_acceleration_covariance[8] = 0;

   /* (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis) */
   //Odo_Msg.pose.covariance = TODO

   /* (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis) */
   //Odo_Msg.twist.covariance = TODO
}

static void Populate_Odo_Msg(void)
{
   Wheel_Speeds_T wheel_ang_v = {0};
   float vr, vl = 0;

   Measure_Wheel_Speeds();
   Get_Wheel_Ang_Velocities(&wheel_ang_v);

   /* Compute the robot's linear wheel velocities */
   vr = wheel_ang_v.r * WHEEL_RADIUS;
   vl = wheel_ang_v.l * WHEEL_RADIUS;

   /* Update message time stamps */
   Odo_Msg.header.stamp = nh.now();

   /* Compute the robot's linear velocity based on the wheel speeds */
   Odo_Msg.twist.twist.linear.x = ((vr + vl)/2);

   /* Compute the robot's angular velocity */
   Odo_Msg.twist.twist.angular.z = ((vr - vl)/WHEEL_BASE);
}

static void Populate_IMU_Msgs(void)
{
   Accel_Data_T   accel_data  = {0};
   Gyro_Data_T    gyro_data   = {0};
   Sensor_Data_T  fxos_data   = {0};

   IMU_Msg_MPU.header.stamp = nh.now();
   IMU_Msg_FXOS.header.stamp = IMU_Msg_MPU.header.stamp;

   My_MPU6050.Read_Accel_Data(&accel_data);
   IMU_Msg_MPU.linear_acceleration.x = accel_data.ax;
   IMU_Msg_MPU.linear_acceleration.y = accel_data.ay;
   IMU_Msg_MPU.linear_acceleration.z = accel_data.az;

   My_MPU6050.Read_Gyro_Data(&gyro_data);
   IMU_Msg_MPU.angular_velocity.x = gyro_data.gx;
   IMU_Msg_MPU.angular_velocity.y = gyro_data.gy;
   IMU_Msg_MPU.angular_velocity.z = gyro_data.gz;

   My_FXOS8700CQ.Read_Data(&fxos_data);
   IMU_Msg_FXOS.linear_acceleration.x = fxos_data.ax;
   IMU_Msg_FXOS.linear_acceleration.y = fxos_data.ay;
   IMU_Msg_FXOS.linear_acceleration.z = fxos_data.az;
}

static inline void Run_GTP_Controller(void)
{
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
}

static void Publish_ROS_Topics(void)
{
   static bool nh_initialized = false;

   if (CONNECTED == Get_Network_Status())
   {
      if (!nh_initialized)
      {
         nh.initNode();
         nh.advertise(imu_mpu);
         nh.advertise(imu_fxos);
         nh.advertise(odo);
         nh_initialized = true;
      }
      else
      {
         imu_mpu.publish(&IMU_Msg_MPU);
         imu_fxos.publish(&IMU_Msg_FXOS);
         odo.publish(&Odo_Msg);
         nh.spinOnce();
      }
   }
}
