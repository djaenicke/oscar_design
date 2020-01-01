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

#define Normalize_14Bits(x) (((x) > (UINT14_MAX/2)) ? (x - UINT14_MAX):(x))
#define Get_14bit_Signed_Val(msb, lsb) ((int16_t)(((uint16_t)((uint16_t)msb << 8) | (uint16_t)lsb) >> 2))
#define Get_16bit_Signed_Val(msb, lsb) ((int16_t)(((uint16_t)((uint16_t)msb << 8) | (uint16_t)lsb)))

const uint8_t FXOS_Dev_Addr[] = {0x1CU, 0x1DU, 0x1EU, 0x1FU};

void FXOS8700CQ::Init(FXOS_Ascale_T ascale)
{
   status_t result = kStatus_Fail;
   uint8_t g_sensor_range = 0;

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

   if (result == kStatus_Success)
   {
      /* Set sensor range */
      Set_Mode(FXOS_STANDBY);
      FXOS_WriteReg(&FXOS_Handle, XYZ_DATA_CFG_REG, (uint8_t) a_scale);

      /* Get sensor range */
      result = FXOS_ReadReg(&FXOS_Handle, XYZ_DATA_CFG_REG, &g_sensor_range, 1);
      Set_Mode(FXOS_ACTIVE);

      if (g_sensor_range != (uint8_t) a_scale)
      {
         assert(0);
      }

      if (result == kStatus_Success)
      {
         Set_Accel_Res((FXOS_Ascale_T) g_sensor_range);
         Calibrate((FXOS_Ascale_T) g_sensor_range);
      }

   }

   if (result != kStatus_Success)
   {
      PRINTF("\r\nFXOS8700CQ initialization failed!\r\n");
      assert(0);
   }
}

void FXOS8700CQ::Set_Accel_Res(FXOS_Ascale_T ascale)
{
   switch (ascale)
   {
      case FXOS_2G:
         scalings.accel = 0.000244 * G;
         break;
      case FXOS_4G:
         scalings.accel = 0.000488 * G;
         break;
      case FXOS_8G:
         scalings.accel = 0.000976 * G;
         break;
      default:
         assert(0);
         break;
   }
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
      destination->ax = temp * scalings.accel;

      temp = Normalize_14Bits(Get_14bit_Signed_Val(sensor_data.accelYMSB, sensor_data.accelYLSB));
      destination->ay = temp * scalings.accel;

      temp = Normalize_14Bits(Get_14bit_Signed_Val(sensor_data.accelZMSB, sensor_data.accelZLSB));
      destination->az = temp * scalings.accel;

      destination->mx = Get_16bit_Signed_Val(sensor_data.magXMSB, sensor_data.magXLSB);
      destination->my = Get_16bit_Signed_Val(sensor_data.magYMSB, sensor_data.magYLSB);
      destination->mz = Get_16bit_Signed_Val(sensor_data.magZMSB, sensor_data.magZLSB);
   }
   else
   {
      destination->data_valid = false;
   }
}

void FXOS8700CQ::Calibrate(FXOS_Ascale_T ascale)
{
   fxos_data_t sensor_data;
   int32_t accel_bias[3] = {0, 0, 0};
   uint8_t divider, temp_u8;

   switch (ascale)
   {
      case FXOS_2G:
         divider = 8;
         break;
      case FXOS_4G:
         divider = 4;
         break;
      case FXOS_8G:
         divider = 2;
         break;
      default:
         assert(0);
         break;
   }

   for (uint8_t i=0; i<NUM_ACCEL_BIAS_SAMPLES; i++)
   {
      FXOS_ReadSensorData(&FXOS_Handle, &sensor_data);

      /* Get the accel data from the sensor data structure in 14 bit left format data */
      accel_bias[X] += Normalize_14Bits(Get_14bit_Signed_Val(sensor_data.accelXMSB, sensor_data.accelXLSB));
      accel_bias[Y] += Normalize_14Bits(Get_14bit_Signed_Val(sensor_data.accelYMSB, sensor_data.accelYLSB));
      accel_bias[Z] += Normalize_14Bits(Get_14bit_Signed_Val(sensor_data.accelZMSB, sensor_data.accelZLSB));

      Delay(1);
   }

   accel_bias[X] /= NUM_ACCEL_BIAS_SAMPLES;
   accel_bias[Y] /= NUM_ACCEL_BIAS_SAMPLES;
   accel_bias[Z] /= NUM_ACCEL_BIAS_SAMPLES;

   Set_Mode(FXOS_STANDBY);

   temp_u8 = (uint8_t)((-accel_bias[X])/divider);
   FXOS_WriteReg(&FXOS_Handle, OFF_X_REG, temp_u8);

   temp_u8 = (uint8_t)((-accel_bias[Y])/divider);
   FXOS_WriteReg(&FXOS_Handle, OFF_Y_REG, temp_u8);

   temp_u8 = (uint8_t) (-1*((1/(scalings.accel/G)) - (accel_bias[Z]/divider)));
   FXOS_WriteReg(&FXOS_Handle, OFF_Z_REG, temp_u8);

   // Output scaled accelerometer biases for manual subtraction in the main program
   biases.accel[X] = (float) accel_bias[X] * scalings.accel;
   biases.accel[Y] = (float) accel_bias[Y] * scalings.accel;
   biases.accel[Z] = G - ((float) accel_bias[Z] * scalings.accel);

   Set_Mode(FXOS_ACTIVE);
}

void FXOS8700CQ::Set_Mode(FXOS_Mode_T mode)
{
   uint8_t desired_ctrl_reg1, current_ctrl_reg1;

   FXOS_ReadReg(&FXOS_Handle, CTRL_REG1, &current_ctrl_reg1, 1);

   switch (mode)
   {
      case FXOS_STANDBY:
         desired_ctrl_reg1 = current_ctrl_reg1 & ~0x01;
         break;
      case FXOS_ACTIVE:
         desired_ctrl_reg1 = current_ctrl_reg1 | 0x01;
         break;
   }

   FXOS_WriteReg(&FXOS_Handle, CTRL_REG1, desired_ctrl_reg1);

   do
   {
      Delay(1);
      FXOS_ReadReg(&FXOS_Handle, CTRL_REG1, &current_ctrl_reg1, 1);
   } while (current_ctrl_reg1 != desired_ctrl_reg1);
}
