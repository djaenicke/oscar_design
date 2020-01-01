/*
 * FXOS8700CQ.h
 *
 *  Created on: Jul 19, 2019
 *      Author: Devin
 */

#ifndef FXOS8700CQ_H_
#define FXOS8700CQ_H_

#include <stdint.h>

#include "fsl_fxos.h"
#include "i2c_fxos.h"

typedef struct {
   float accel[3];
   float magno[3];
} FXOS_Meas_Biases_T;

typedef struct {
   float accel;
   float magno;
} FXOS_Meas_Scalings_T;

typedef struct
{
   bool data_valid;
   float ax;
   float ay;
   float az;
   float mx;
   float my;
   float mz;
} Sensor_Data_T;

typedef enum
{
  FXOS_2G = 0,
  FXOS_4G,
  FXOS_8G,
} FXOS_Ascale_T;

typedef enum
{
  FXOS_STANDBY = 0x00,
  FXOS_ACTIVE = 0x01
} FXOS_Mode_T;

class FXOS8700CQ:I2C_FXOS
{
private:
   FXOS_Ascale_T a_scale;
   FXOS_Meas_Biases_T biases;
   FXOS_Meas_Scalings_T scalings;

   fxos_handle_t FXOS_Handle = {0};
   fxos_config_t FXOS_Cfg    = {0};

   void Set_Mode(FXOS_Mode_T mode);
   void Set_Accel_Res(FXOS_Ascale_T ascale);
   void Calibrate(FXOS_Ascale_T ascale);

public:
   void Init(FXOS_Ascale_T ascale);
   void Read_Data(Sensor_Data_T * destination);
};

#endif /* FXOS8700CQ_H_ */
