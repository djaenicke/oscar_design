/*
 * ultrasonic_sensor.h
 *
 *  Created on: Jul 4, 2019
 *      Author: Devin
 */

#ifndef ULTRASONIC_SENSOR_H_
#define ULTRASONIC_SENSOR_H_

typedef enum
{
   IDLE = 0,
   TRIG,
   ECHO
} Sensor_State_T;

class UltrasonicSensor
{
private:
   Sensor_State_T state;
public:
   void Init(void);
   void Trigger(void);

   Sensor_State_T Get_State(void);
   void Set_State(Sensor_State_T new_state);
};

#endif /* ULTRASONIC_SENSOR_H_ */
