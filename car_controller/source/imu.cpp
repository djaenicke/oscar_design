/*
 * imu.cpp
 *
 *  Created on: Jul 7, 2019
 *      Author: Devin
 */

#include "imu.h"
#include "fsl_i2c.h"
#include "fsl_fxos.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_debug_console.h"
#include "clock_config.h"
#include "mpu6050.h"

#define ACCEL_I2C_CLOCK_FREQ  CLOCK_GetFreq(I2C0_CLK_SRC)
#define I2C_RELEASE_BUS_COUNT 100U
#define I2C_RELEASE_SDA_PORT  PORTE
#define I2C_RELEASE_SCL_PORT  PORTE
#define I2C_RELEASE_SDA_GPIO  GPIOE
#define I2C_RELEASE_SDA_PIN   25U
#define I2C_RELEASE_SCL_GPIO  GPIOE
#define I2C_RELEASE_SCL_PIN   24U

#define G (9.81f)

static MPU6050 My_MPU6050;
static fxos_handle_t FXOS_Handle = {0};
static fxos_data_t   Sensor_Data = {0};
static fxos_config_t FXOS_Cfg    = {0};
static const uint8_t FXOS_Dev_Addr[] = {0x1CU, 0x1DU, 0x1EU, 0x1FU};

static Accel_Data_T Onboard_Accel_Data = {0};
static Accel_Data_T MPU6050_Accel_Data = {0};

static float Onboard_Accel_Scaling = 0.0f;
static float MPU6050_Accel_Scaling = 0.0f;

static inline void Init_Onboard_IMU(void);
static void Read_Accel_Data(void);
static inline void I2C_Release_Bus(void);
static void I2C_Release_Bus_Delay(void);
static inline void I2C_Configure_Pins(void);
status_t IMU_I2C_Tx(uint8_t device_addr, uint32_t sub_addr, uint8_t sub_addr_size, uint32_t tx_buff);
status_t IMU_I2C_Rx(uint8_t dev_addr, uint32_t sub_addr, uint8_t sub_addr_size, \
                    uint8_t *rx_buff, uint8_t rx_buff_size);

void Init_IMU(void)
{
   float res1[3], res2[3], res3[6];

   /* Initialize the external 6-axis MPU6050 */
   My_MPU6050.Set_FTM(FTM0);
   My_MPU6050.Init_I2C(I2C1);
   My_MPU6050.Calibrate(res1, res2);
   My_MPU6050.Run_Self_Test(res3);
   MPU6050_Accel_Scaling = My_MPU6050.Get_Accel_Res() * G;

   /* Initialize the 6-axis on board IMU */
   Init_Onboard_IMU();

   Read_Accel_Data();
}

static inline void Init_Onboard_IMU(void)
{
   i2c_master_config_t i2c_cfg = {0};
   uint8_t array_addr_size     = 0;
   status_t result             = kStatus_Fail;
   uint8_t sensor_range        = 0;

   I2C_Release_Bus();
   I2C_Configure_Pins();
   I2C_MasterGetDefaultConfig(&i2c_cfg);
   I2C_MasterInit(I2C0, &i2c_cfg, ACCEL_I2C_CLOCK_FREQ);

   FXOS_Cfg.I2C_SendFunc    = IMU_I2C_Tx;
   FXOS_Cfg.I2C_ReceiveFunc = IMU_I2C_Rx;

   array_addr_size = sizeof(FXOS_Dev_Addr) / sizeof(FXOS_Dev_Addr[0]);
   for (uint8_t i = 0; i < array_addr_size; i++)
   {
      FXOS_Cfg.slaveAddress = FXOS_Dev_Addr[i];
      /* Initialize accelerometer sensor */
      result = FXOS_Init(&FXOS_Handle, &FXOS_Cfg);
      if (kStatus_Success == result)
      {
         break;
      }
   }

   if (result != kStatus_Success)
   {
      PRINTF("\r\nIMU initialize failed!\r\n");
   }

   /* Get sensor range */
   if (FXOS_ReadReg(&FXOS_Handle, XYZ_DATA_CFG_REG, &sensor_range, 1) != kStatus_Success)
   {
      assert(false);
   }

   if (0x00 == sensor_range)
   {
      Onboard_Accel_Scaling = (2.0f/8192.0f) * G;
   }
   else if (0x01 == sensor_range)
   {
      Onboard_Accel_Scaling = (4.0f/8192.0f) * G;
   }
   else if (0x10 == sensor_range)
   {
      Onboard_Accel_Scaling = (8.0f/8192.0f) * G;
   }
   else
   {
      assert(false);
   }
}

static void Read_Accel_Data(void)
{
   int16_t a[3];

   /* Get readings from onboard sensor */
   if (FXOS_ReadSensorData(&FXOS_Handle, &Sensor_Data) != kStatus_Success)
   {
      assert(false);
   }

   a[0] = (int16_t)((uint16_t)((uint16_t)Sensor_Data.accelXMSB << 8) | (uint16_t)Sensor_Data.accelXLSB) / 4U;
   a[1] = (int16_t)((uint16_t)((uint16_t)Sensor_Data.accelYMSB << 8) | (uint16_t)Sensor_Data.accelYLSB) / 4U;
   a[2] = (int16_t)((uint16_t)((uint16_t)Sensor_Data.accelZMSB << 8) | (uint16_t)Sensor_Data.accelZLSB) / 4U;

   Onboard_Accel_Data.ax = a[0] * Onboard_Accel_Scaling;
   Onboard_Accel_Data.ay = a[1] * Onboard_Accel_Scaling;
   Onboard_Accel_Data.az = a[2] * Onboard_Accel_Scaling;

   /* Get readings from MPU6050 sensor */
   My_MPU6050.Read_Accel_Data(a);

   MPU6050_Accel_Data.ax = a[0] * MPU6050_Accel_Scaling;
   MPU6050_Accel_Data.ay = a[1] * MPU6050_Accel_Scaling;
   MPU6050_Accel_Data.az = a[2] * MPU6050_Accel_Scaling;
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

status_t IMU_I2C_Tx(uint8_t device_addr, uint32_t sub_addr, uint8_t sub_addr_size, uint32_t tx_buff)
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

status_t IMU_I2C_Rx(uint8_t dev_addr, uint32_t sub_addr, uint8_t sub_addr_size, \
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

