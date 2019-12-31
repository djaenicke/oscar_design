/*
 * i2c_fxos.h
 *
 *  Created on: Dec 31, 2019
 *      Author: Devin
 */

#ifndef HW_ABSTRACTION_I2C_FXOS_H_
#define HW_ABSTRACTION_I2C_FXOS_H_

#include <stdint.h>

#include "fsl_i2c.h"
#include "fsl_common.h"

class I2C_FXOS
{
private:
   I2C_Type * I2C_Base;

public:
   static status_t I2C_Tx(uint8_t device_addr, uint32_t sub_addr, uint8_t sub_addr_size, uint32_t tx_buff)
   {
       uint8_t data = (uint8_t)tx_buff;
       i2c_master_transfer_t masterXfer;

       /* Prepare transfer structure. */
       masterXfer.slaveAddress   = device_addr;
       masterXfer.direction      = kI2C_Write;
       masterXfer.subaddress     = sub_addr;
       masterXfer.subaddressSize = sub_addr_size;
       masterXfer.data           = &data;
       masterXfer.dataSize       = 1;
       masterXfer.flags          = kI2C_TransferDefaultFlag;

       return I2C_MasterTransferBlocking(I2C0, &masterXfer);
   }

   static status_t I2C_Rx(uint8_t dev_addr, uint32_t sub_addr, uint8_t sub_addr_size, \
                       uint8_t *rx_buff, uint8_t rx_buff_size)
   {
      i2c_master_transfer_t master_xfer;

      /* Prepare transfer structure. */
      master_xfer.slaveAddress   = dev_addr;
      master_xfer.subaddress     = sub_addr;
      master_xfer.subaddressSize = sub_addr_size;
      master_xfer.data           = rx_buff;
      master_xfer.dataSize       = rx_buff_size;
      master_xfer.direction      = kI2C_Read;
      master_xfer.flags          = kI2C_TransferDefaultFlag;

      return I2C_MasterTransferBlocking(I2C0, &master_xfer);
   }
};


#endif /* HW_ABSTRACTION_I2C_FXOS_H_ */
