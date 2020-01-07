/*
 * FXOS8700CQ.cpp
 *
 *  Created on: Jul 19, 2019
 *      Author: Devin
 */

#include "FXOS8700CQ.h"
#include "fsl_debug_console.h"

#define X 0
#define Y 1
#define Z 2
#define G (9.81f)

#define UINT14_MAX  (0x3FFF)
#define NUM_ACCEL_BIAS_SAMPLES ((uint8_t)100)

#define SENSITIVITY_2G 4096
#define SENSITIVITY_4G 2048
#define SENSITIVITY_8G 1024

#define Normalize_14Bits(x) (((x) > (UINT14_MAX/2)) ? (x - UINT14_MAX):(x))
#define Get_14bit_Signed_Val(msb, lsb) ((int16_t)(((uint16_t)((uint16_t)msb << 8) | (uint16_t)lsb) >> 2))
#define Get_16bit_Signed_Val(msb, lsb) ((int16_t)(((uint16_t)((uint16_t)msb << 8) | (uint16_t)lsb)))

const uint8_t FXOS_Dev_Addr[] = {0x1CU, 0x1DU, 0x1EU, 0x1FU};

void FXOS8700CQ::Init(FXOS_Ascale_T ascale)
{
   status_t result = kStatus_Fail;
   a_scale = ascale;

   Init_I2C_If();

   FXOS_Cfg.I2C_SendFunc    = I2C_Tx;
   FXOS_Cfg.I2C_ReceiveFunc = I2C_Rx;

   for (uint8_t i = 0; i < sizeof(FXOS_Dev_Addr) / sizeof(FXOS_Dev_Addr[0]); i++)
   {
      FXOS_Cfg.slaveAddress = FXOS_Dev_Addr[i];
      result = FXOS_Init(&FXOS_Handle, &FXOS_Cfg);

      if (kStatus_Success == result)
      {
         break;
      }
   }

   assert(result == kStatus_Success);

   Set_Ascale();
   Set_ODR(FXOS_50HZ);
   Enable_Reduced_Noise();
   Set_Mode(FXOS_STANDBY);
   FXOS_WriteReg(&FXOS_Handle, CTRL_REG2, 0x02); // High Resolution mode
   Set_Mode(FXOS_ACTIVE);
   Calibrate();
}

void FXOS8700CQ::Set_Accel_Res(FXOS_Ascale_T ascale)
{
   switch (ascale)
   {
      case FXOS_2G:
         accel_sensitivity = SENSITIVITY_2G;
         break;
      case FXOS_4G:
         accel_sensitivity = SENSITIVITY_4G;
         break;
      case FXOS_8G:
         accel_sensitivity = SENSITIVITY_8G;
         break;
      default:
         assert(0);
         break;
   }

   scalings.accel = (1.0/accel_sensitivity) * G;
}

void FXOS8700CQ::Read_Data(Sensor_Data_T * destination)
{
   fxos_data_t sensor_data;
   int16_t temp;

   if (kStatus_Success == FXOS_ReadSensorData(&FXOS_Handle, &sensor_data))
   {
      destination->data_valid = true;

      /* Get the accel data from the sensor data structure in 14 bit left format data */
      temp = Normalize_14Bits(Get_14bit_Signed_Val(sensor_data.accelXMSB, sensor_data.accelXLSB));
      destination->ax = (temp * scalings.accel) - biases.accel[X];

      temp = Normalize_14Bits(Get_14bit_Signed_Val(sensor_data.accelYMSB, sensor_data.accelYLSB));
      destination->ay = (temp * scalings.accel) - biases.accel[Y];

      temp = Normalize_14Bits(Get_14bit_Signed_Val(sensor_data.accelZMSB, sensor_data.accelZLSB));
      destination->az = (temp * scalings.accel) - biases.accel[Z];

      destination->mx = Get_16bit_Signed_Val(sensor_data.magXMSB, sensor_data.magXLSB);
      destination->my = Get_16bit_Signed_Val(sensor_data.magYMSB, sensor_data.magYLSB);
      destination->mz = Get_16bit_Signed_Val(sensor_data.magZMSB, sensor_data.magZLSB);
   }
   else
   {
      destination->data_valid = false;
   }
}

void FXOS8700CQ::Calibrate(void)
{
   fxos_data_t sensor_data_raw;
   int32_t accel_bias[3] = {0, 0, 0};

   for (uint8_t i=0; i<NUM_ACCEL_BIAS_SAMPLES; i++)
   {
      FXOS_ReadSensorData(&FXOS_Handle, &sensor_data_raw);

      /* Get the accel data from the sensor data structure in 14 bit left format data */
      accel_bias[X] += Normalize_14Bits(Get_14bit_Signed_Val(sensor_data_raw.accelXMSB, sensor_data_raw.accelXLSB));
      accel_bias[Y] += Normalize_14Bits(Get_14bit_Signed_Val(sensor_data_raw.accelYMSB, sensor_data_raw.accelYLSB));
      accel_bias[Z] += Normalize_14Bits(Get_14bit_Signed_Val(sensor_data_raw.accelZMSB, sensor_data_raw.accelZLSB));

      Delay(1);
   }

   accel_bias[X] /= NUM_ACCEL_BIAS_SAMPLES;
   accel_bias[Y] /= NUM_ACCEL_BIAS_SAMPLES;
   accel_bias[Z] /= NUM_ACCEL_BIAS_SAMPLES;

   biases.accel[X] = accel_bias[X] * scalings.accel;
   biases.accel[Y] = accel_bias[Y] * scalings.accel;
   biases.accel[Z] = (accel_bias[Z] * scalings.accel) - G;
}

void FXOS8700CQ::Set_ODR(ODR_Hybrid_T odr)
{
   CTRL_1_T desired_ctrl_reg1;

   Set_Mode(FXOS_STANDBY);

   FXOS_ReadReg(&FXOS_Handle, CTRL_REG1, &desired_ctrl_reg1.byte, 1);
   desired_ctrl_reg1.odr = odr;
   FXOS_WriteReg(&FXOS_Handle, CTRL_REG1, desired_ctrl_reg1.byte);

   Set_Mode(FXOS_ACTIVE);
}

void FXOS8700CQ::Enable_Reduced_Noise(void)
{
   CTRL_1_T desired_ctrl_reg1;

   Set_Mode(FXOS_STANDBY);

   FXOS_ReadReg(&FXOS_Handle, CTRL_REG1, &desired_ctrl_reg1.byte, 1);
   desired_ctrl_reg1.lnoise = 1;
   FXOS_WriteReg(&FXOS_Handle, CTRL_REG1, desired_ctrl_reg1.byte);

   Set_Mode(FXOS_ACTIVE);
}

void FXOS8700CQ::Set_Mode(FXOS_Mode_T mode)
{
   CTRL_1_T desired_ctrl_reg1;

   FXOS_ReadReg(&FXOS_Handle, CTRL_REG1, &desired_ctrl_reg1.byte, 1);
   desired_ctrl_reg1.active = mode;
   FXOS_WriteReg(&FXOS_Handle, CTRL_REG1, desired_ctrl_reg1.byte);

   do
   {
      Delay(1);
      FXOS_ReadReg(&FXOS_Handle, CTRL_REG1, &desired_ctrl_reg1.byte, 1);
   } while (mode != desired_ctrl_reg1.active);
}

void FXOS8700CQ::Set_Ascale(void)
{
   uint8_t g_sensor_range;

   Set_Mode(FXOS_STANDBY);

   FXOS_WriteReg(&FXOS_Handle, XYZ_DATA_CFG_REG, (uint8_t) a_scale);

   do
   {
      Delay(1);
      FXOS_ReadReg(&FXOS_Handle, XYZ_DATA_CFG_REG, &g_sensor_range, 1);
   } while (g_sensor_range != (uint8_t) a_scale);

   Set_Accel_Res((FXOS_Ascale_T) g_sensor_range);

   Set_Mode(FXOS_ACTIVE);
}

CTRL_1_T FXOS8700CQ::Read_CTRL_REG1(void)
{
   CTRL_1_T ctrl_reg1;

   FXOS_ReadReg(&FXOS_Handle, CTRL_REG1, &ctrl_reg1.byte, 1);

   return(ctrl_reg1);
}
