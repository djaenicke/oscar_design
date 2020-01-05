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

typedef union
{
   struct {
   uint8_t active:1;
   uint8_t f_read:1;
   uint8_t lnoise:1;
   uint8_t odr:3;
   uint8_t aslp_rate:2;
   };
   uint8_t byte;
} CTRL_1_T;

typedef enum {
   FXOS_400HZ    = 0,
   FXOS_200HZ    = 1,
   FXOS_100HZ    = 2,
   FXOS_50HZ     = 3,
   FXOS_25HZ     = 4,
   FXOS_6p25HZ   = 5,
   FXOS_3p125HZ  = 6,
   FXOS_0p7813HZ = 7,
} ODR_Hybrid_T;

class FXOS8700CQ:I2C_FXOS
{
private:
   FXOS_Ascale_T a_scale;
   FXOS_Meas_Biases_T biases;
   FXOS_Meas_Scalings_T scalings;
   uint16_t accel_sensitivity;

   fxos_handle_t FXOS_Handle = {0};
   fxos_config_t FXOS_Cfg    = {0};

   void Set_Mode(FXOS_Mode_T mode);
   void Set_Ascale(void);
   void Set_ODR(ODR_Hybrid_T odr);
   void Enable_Reduced_Noise(void);
   void Set_Accel_Res(FXOS_Ascale_T ascale);
   void Calibrate(void);
   CTRL_1_T Read_CTRL_REG1(void);

public:
   void Init(FXOS_Ascale_T ascale);
   void Read_Data(Sensor_Data_T * destination);
};

#endif /* FXOS8700CQ_H_ */
