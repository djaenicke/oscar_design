/*
 * ultrasonic_sensor.h
 *
 *  Created on: Jul 4, 2019
 *      Author: Devin
 */

#ifndef ULTRASONIC_SENSOR_H_
#define ULTRASONIC_SENSOR_H_

#include "fsl_ftm.h"
#include "io_abstraction.h"

typedef enum
{
   IDLE = 0,
   TRIG,
   ECHO
} Sensor_State_T;

typedef struct {
   Sensor_State_T state;
   FTM_Type *ftm_ptr;
   IO_Map_T trig;
   IO_Map_T echo;
} USS_Working_Info_T;

class UltrasonicSensor
{
private:
   USS_Working_Info_T working_info;
public:
   void Init(FTM_Type *ftm_base_ptr, IO_Map_T trig_pin, IO_Map_T echo_pin);
   void Trigger(void);
};

#endif /* ULTRASONIC_SENSOR_H_ */
