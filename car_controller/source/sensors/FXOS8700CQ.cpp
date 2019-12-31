/*
 * FXOS8700CQ.cpp
 *
 *  Created on: Jul 19, 2019
 *      Author: Devin
 */

#include "FXOS8700CQ.h"
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

#define G (9.81f)
#define UINT14_MAX  (0x3FFF)

#define Normalize_14Bits(x) (((x) > (UINT14_MAX/2)) ? (x - UINT14_MAX):(x))
#define Get_14bit_Signed_Val(msb, lsb) ((int16_t)(((uint16_t)((uint16_t)msb << 8) | (uint16_t)lsb) >> 2))
#define Get_16bit_Signed_Val(msb, lsb) ((int16_t)(((uint16_t)((uint16_t)msb << 8) | (uint16_t)lsb)))

static inline void I2C_Release_Bus(void);
static void I2C_Release_Bus_Delay(void);
static inline void I2C_Configure_Pins(void);

const uint8_t FXOS_Dev_Addr[] = {0x1CU, 0x1DU, 0x1EU, 0x1FU};

void FXOS8700CQ::Init(void)
{
   i2c_master_config_t i2c_cfg = {0};
   uint8_t array_addr_size     = 0;
   status_t result             = kStatus_Fail;
   uint8_t g_sensor_range;

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

   if (result == kStatus_Success)
   {
      /* Get sensor range */
      if (FXOS_ReadReg(&FXOS_Handle, XYZ_DATA_CFG_REG, &g_sensor_range, 1) != kStatus_Success)
      {
         PRINTF("\r\nFXOS8700CQ initialization failed!\r\n");
         assert(0);
      }

      Set_Accel_Res((FXOS_Ascale_T)g_sensor_range);
   }
   else
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
   status_t result = kStatus_Fail;
   fxos_data_t sensor_data;
   int16_t temp;

   result = FXOS_ReadSensorData(&FXOS_Handle, &sensor_data);

   if (kStatus_Success == result)
   {
      /* Get the accel data from the sensor data structure in 14 bit left format data */
      temp = Get_14bit_Signed_Val(sensor_data.accelXMSB, sensor_data.accelXLSB);
      temp = Normalize_14Bits(temp);
      destination->ax = temp * scalings.accel;

      temp = Get_14bit_Signed_Val(sensor_data.accelYMSB, sensor_data.accelYLSB);
      temp = Normalize_14Bits(temp);
      destination->ay = temp * scalings.accel;

      temp = Get_14bit_Signed_Val(sensor_data.accelZMSB, sensor_data.accelZLSB);
      temp = Normalize_14Bits(temp);
      destination->az = temp * scalings.accel;

      destination->mx = Get_16bit_Signed_Val(sensor_data.magXMSB, sensor_data.magXLSB);
      destination->my = Get_16bit_Signed_Val(sensor_data.magYMSB, sensor_data.magYLSB);
      destination->mz = Get_16bit_Signed_Val(sensor_data.magZMSB, sensor_data.magZLSB);
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
