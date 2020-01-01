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
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "clock_config.h"
#include "delay.h"

#define ACCEL_I2C_CLOCK_FREQ  CLOCK_GetFreq(I2C0_CLK_SRC)
#define I2C_RELEASE_BUS_COUNT 100U
#define I2C_RELEASE_SDA_PORT  PORTE
#define I2C_RELEASE_SCL_PORT  PORTE
#define I2C_RELEASE_SDA_GPIO  GPIOE
#define I2C_RELEASE_SDA_PIN   25U
#define I2C_RELEASE_SCL_GPIO  GPIOE
#define I2C_RELEASE_SCL_PIN   24U

class I2C_FXOS
{
private:

public:
   void Init_I2C_If(void)
   {
      i2c_master_config_t i2c_cfg = {0};

      I2C_Release_Bus();
      I2C_Configure_Pins();
      I2C_MasterGetDefaultConfig(&i2c_cfg);
      I2C_MasterInit(I2C0, &i2c_cfg, ACCEL_I2C_CLOCK_FREQ);
   }

   void Delay(uint16_t delay_ms)
   {
      Delay_ms(delay_ms);
   }

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

   static void I2C_Release_Bus_Delay(void)
   {
      uint32_t i = 0;
      for (i = 0; i < I2C_RELEASE_BUS_COUNT; i++)
      {
         __NOP();
      }
   }

   void I2C_Release_Bus(void)
   {
      uint8_t i = 0;

      gpio_pin_config_t pin_config;
      port_pin_config_t i2c_pin_config = {0};

      /* Config pin mux as gpio */
      i2c_pin_config.pullSelect = kPORT_PullUp;
      i2c_pin_config.mux        = kPORT_MuxAsGpio;

      pin_config.pinDirection = kGPIO_DigitalOutput;
      pin_config.outputLogic  = 1U;

      PORT_SetPinConfig(I2C_RELEASE_SCL_PORT, I2C_RELEASE_SCL_PIN, &i2c_pin_config);
      PORT_SetPinConfig(I2C_RELEASE_SDA_PORT, I2C_RELEASE_SDA_PIN, &i2c_pin_config);

      GPIO_PinInit(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, &pin_config);
      GPIO_PinInit(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, &pin_config);

      /* Drive SDA low first to simulate a start */
      GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
      I2C_Release_Bus_Delay();

      /* Send 9 pulses on SCL and keep SDA high */
      for (i = 0; i < 9; i++)
      {
        GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
        I2C_Release_Bus_Delay();

        GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
        I2C_Release_Bus_Delay();

        GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
        I2C_Release_Bus_Delay();
        I2C_Release_Bus_Delay();
      }

      /* Send stop */
      GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
      I2C_Release_Bus_Delay();

      GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
      I2C_Release_Bus_Delay();

      GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
      I2C_Release_Bus_Delay();

      GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
      I2C_Release_Bus_Delay();
   }

   void I2C_Configure_Pins(void)
   {
      /* IMU pin configurations */
      const port_pin_config_t porte24_pin31_config = {
        kPORT_PullUp,                                            /* Internal pull-up resistor is enabled */
        kPORT_FastSlewRate,                                      /* Fast slew rate is configured */
        kPORT_PassiveFilterDisable,                              /* Passive filter is disabled */
        kPORT_OpenDrainEnable,                                   /* Open drain is enabled */
        kPORT_LowDriveStrength,                                  /* Low drive strength is configured */
        kPORT_MuxAlt5,                                           /* Pin is configured as I2C0_SCL */
        kPORT_UnlockRegister                                     /* Pin Control Register fields [15:0] are not locked */
      };
      PORT_SetPinConfig(PORTE, 24, &porte24_pin31_config); /* PORTE24 (pin 31) is configured as I2C0_SCL */
      const port_pin_config_t porte25_pin32_config = {
        kPORT_PullUp,                                            /* Internal pull-up resistor is enabled */
        kPORT_FastSlewRate,                                      /* Fast slew rate is configured */
        kPORT_PassiveFilterDisable,                              /* Passive filter is disabled */
        kPORT_OpenDrainEnable,                                   /* Open drain is enabled */
        kPORT_LowDriveStrength,                                  /* Low drive strength is configured */
        kPORT_MuxAlt5,                                           /* Pin is configured as I2C0_SDA */
        kPORT_UnlockRegister                                     /* Pin Control Register fields [15:0] are not locked */
      };
      PORT_SetPinConfig(PORTE, 25, &porte25_pin32_config); /* PORTE25 (pin 32) is configured as I2C0_SDA */
   }
};


#endif /* HW_ABSTRACTION_I2C_FXOS_H_ */
