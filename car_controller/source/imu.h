/*
 * imu.h
 *
 *  Created on: Jul 7, 2019
 *      Author: Devin
 */

#ifndef IMU_H_
#define IMU_H_

typedef struct {
   float ax;
   float ay;
   float az;
} Accel_Data_T;

extern void Init_IMU(void);

#endif /* IMU_H_ */
