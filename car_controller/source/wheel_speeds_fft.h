/*
 * wheel_speeds_fft.h
 *
 *  Created on: Dec 1, 2019
 *      Author: Devin
 */

#ifndef WHEEL_SPEEDS_FFT_H_
#define WHEEL_SPEEDS_FFT_H_

#include "wheel_speeds.h"

extern void Init_Wheel_Speeds_FFT(void);
extern void Get_Wheel_Speeds_FFT(Wheel_Speeds_T * speeds);
extern void Zero_Wheel_Speed_FFT(Wheel_Sensor_T sensor);

#endif /* WHEEL_SPEEDS_FFT_H_ */
