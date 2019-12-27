/*
 * FXOS8700CQ.cpp
 *
 *  Created on: Jul 19, 2019
 *      Author: Devin
 */

#include "FXOS8700CQ.h"
#include "fsl_i2c.h"
#include "fsl_fxos.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_debug_console.h"
#include "clock_config.h"

#define ACCEL_I2C_CLOCK_FREQ  CLOCK_GetFreq(I2C0_CLK_SRC)
#define I2C_RELEASE_BUS_COUNT 100U
#define I2C_RELEASE_SDA_PORT  PORTE
#define I2C_RELEASE_SCL_PORT  PORTE
#define I2C_RELEASE_SDA_GPIO  GPIOE
#define I2C_RELEASE_SDA_PIN   25U
#define I2C_RELEASE_SCL_GPIO  GPIOE
#define I2C_RELEASE_SCL_PIN   24U

static fxos_handle_t FXOS_Handle = {0};
static fxos_config_t FXOS_Cfg    = {0};
static const uint8_t FXOS_Dev_Addr[] = {0x1CU, 0x1DU, 0x1EU, 0x1FU};

static inline void I2C_Release_Bus(void);
static void I2C_Release_Bus_Delay(void);
static inline void I2C_Configure_Pins(void);

status_t I2C_Tx(uint8_t device_addr, uint32_t sub_addr, uint8_t sub_addr_size, uint32_t tx_buff);
status_t I2C_Rx(uint8_t dev_addr, uint32_t sub_addr, uint8_t sub_addr_size, \
                           uint8_t *rx_buff, uint8_t rx_buff_size);

void FXOS8700CQ::Init(void)
{
   i2c_master_config_t i2c_cfg = {0};
   uint8_t array_addr_size     = 0;
   status_t result             = kStatus_Fail;

   I2C_Release_Bus();
   I2C_Configure_Pins();
   I2C_MasterGetDefaultConfig(&i2c_cfg);
   I2C_MasterInit(I2C0, &i2c_cfg, ACCEL_I2C_CLOCK_FREQ);

   FXOS_Cfg.I2C_SendFunc    = I2C_Tx;
   FXOS_Cfg.I2C_ReceiveFunc = I2C_Rx;

   array_addr_size = sizeof(FXOS_Dev_Addr) / sizeof(FXOS_Dev_Addr[0]);
   for (uint8_t i = 0; i < array_addr_size; i++)
   {
      FXOS_Cfg.slaveAddress = FXOS_Dev_Addr[i];
      result = FXOS_Init(&FXOS_Handle, &FXOS_Cfg);
      if (kStatus_Success == result)
      {
         break;
      }
   }

   if (result != kStatus_Success)
   {
      PRINTF("\r\nFXOS8700CQ initialization failed!\r\n");
   }
}

static void I2C_Release_Bus_Delay(void)
{
   uint32_t i = 0;
   for (i = 0; i < I2C_RELEASE_BUS_COUNT; i++)
   {
      __NOP();
   }
}

static inline void I2C_Release_Bus(void)
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

static inline void I2C_Configure_Pins(void)
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

status_t I2C_Tx(uint8_t device_addr, uint32_t sub_addr, uint8_t sub_addr_size, uint32_t tx_buff)
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

status_t I2C_Rx(uint8_t dev_addr, uint32_t sub_addr, uint8_t sub_addr_size, \
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

