/*
 * inertial_states.cpp
 *
 *  Created on: Jul 7, 2019
 *      Author: Devin
 */

#include "inertial_states.h"
#include "mpu6050.h"
#include "FXOS8700CQ.h"

static MPU6050    My_MPU6050;
static FXOS8700CQ My_FXOS8700CQ;

static Accel_Data_T MPU6050_Accel_Data = {0};
static Gyro_Data_T  MPU6050_Gyro_Data  = {0};

void Init_Inertial_Sensors(void)
{
   /* Initialize the external 6-axis MPU6050 */
   My_MPU6050.Init(FTM0, I2C1);

   /* Initialize the 6-axis on board FXOS8700CQ */
   My_FXOS8700CQ.Init();

   /* Get readings from MPU6050 sensor */
   My_MPU6050.Read_Accel_Data(&MPU6050_Accel_Data);
   My_MPU6050.Read_Gyro_Data(&MPU6050_Gyro_Data);
}

