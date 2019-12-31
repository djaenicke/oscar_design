/*
 * i2c_mpu6050.h
 *
 *  Created on: Dec 31, 2019
 *      Author: Devin
 */

#ifndef HW_ABSTRACTION_I2C_MPU6050_H_
#define HW_ABSTRACTION_I2C_MPU6050_H_

#include <stdint.h>

#include "fsl_i2c.h"
#include "fsl_common.h"
#include "delay.h"

class I2C_MPU6050
{
private:
   I2C_Type * I2C_Base;

public:
   void Init_I2C_If(void)
   {
      i2c_master_config_t i2c_cfg = {0};

      /* Initialize I2C */
      I2C_Base = I2C1;
      I2C_MasterGetDefaultConfig(&i2c_cfg);
      I2C_MasterInit(I2C_Base, &i2c_cfg, CLOCK_GetFreq(kCLOCK_BusClk));
   }

   void Delay(uint16_t delay_ms)
   {
      Delay_ms(delay_ms);
   }

   void Write_Byte(uint8_t addr, uint8_t sub_addr, uint8_t data)
   {
      status_t status;
      i2c_master_transfer_t masterXfer;

      /* Prepare transfer structure. */
      masterXfer.slaveAddress   = addr;
      masterXfer.direction      = kI2C_Write;
      masterXfer.subaddress     = sub_addr;
      masterXfer.subaddressSize = 1;
      masterXfer.data           = &data;
      masterXfer.dataSize       = 1;
      masterXfer.flags          = kI2C_TransferDefaultFlag;

      status = I2C_MasterTransferBlocking(I2C_Base, &masterXfer);

      switch(status)
      {
         case kStatus_Success:
            break;
         case kStatus_I2C_Busy:
            assert(false);
            break;
         case kStatus_I2C_Timeout:
            assert(false);
            break;
         case kStatus_I2C_ArbitrationLost:
            assert(false);
            break;
         case kStatus_I2C_Nak:
            assert(false);
            break;
         default:
            assert(false);
            break;
      }
   }

   void Read_Bytes(uint8_t addr, uint8_t sub_addr, uint8_t count, uint8_t * dest)
   {
      status_t status;
      i2c_master_transfer_t master_xfer;

      /* Prepare transfer structure. */
      master_xfer.slaveAddress   = addr;
      master_xfer.subaddress     = sub_addr;
      master_xfer.subaddressSize = 1;
      master_xfer.data           = dest;
      master_xfer.dataSize       = count;
      master_xfer.direction      = kI2C_Read;
      master_xfer.flags          = kI2C_TransferDefaultFlag;

      status = I2C_MasterTransferBlocking(I2C_Base, &master_xfer);

      switch(status)
      {
         case kStatus_Success:
            break;
         case kStatus_I2C_Busy:
            assert(false);
            break;
         case kStatus_I2C_Timeout:
            assert(false);
            break;
         case kStatus_I2C_ArbitrationLost:
            assert(false);
            break;
         case kStatus_I2C_Nak:
            assert(false);
            break;
         default:
            assert(false);
            break;
      }
   }

   uint8_t Read_Byte(uint8_t addr, uint8_t sub_addr)
   {
      status_t status;
      uint8_t data;
      i2c_master_transfer_t master_xfer;

      /* Prepare transfer structure. */
      master_xfer.slaveAddress   = addr;
      master_xfer.subaddress     = sub_addr;
      master_xfer.subaddressSize = 1;
      master_xfer.data           = &data;
      master_xfer.dataSize       = 1;
      master_xfer.direction      = kI2C_Read;
      master_xfer.flags          = kI2C_TransferDefaultFlag;

      status = I2C_MasterTransferBlocking(I2C_Base, &master_xfer);

      switch(status)
      {
         case kStatus_Success:
            break;
         case kStatus_I2C_Busy:
            assert(false);
            break;
         case kStatus_I2C_Timeout:
            assert(false);
            break;
         case kStatus_I2C_ArbitrationLost:
            assert(false);
            break;
         case kStatus_I2C_Nak:
            assert(false);
            break;
         default:
            assert(false);
            break;
      }

      return data;
   }
};


#endif /* HW_ABSTRACTION_I2C_MPU6050_H_ */
