#include <math.h>

#include "mpu6050.h"
#include "assert.h"
#include "fsl_common.h"

/* Get source clock for FTM driver */
#define FTM_SOURCE_CLOCK (CLOCK_GetFreq(kCLOCK_McgFixedFreqClk)/32)

#define MAX_DELAY (1342) /* ms */

static int Gscale = GFS_250DPS;
static int Ascale = AFS_2G;
static uint32_t FTM_Clk_Freq = 0;

void MPU6050::Init_Delay_Timer(void)
{
   ftm_config_t ftm_info;

   FTM_GetDefaultConfig(&ftm_info);

   /* Divide FTM clock by 4 */
   ftm_info.prescale = kFTM_Prescale_Divide_32;

   FTM_Init(ftm_base, &ftm_info);
   FTM_SetTimerPeriod(ftm_base, UINT16_MAX);

   ftm_base->CNT = 0;

   FTM_Clk_Freq = FTM_SOURCE_CLOCK;
}

void MPU6050::Delay(uint16_t ms_delay)
{
   uint32_t start_cnt;
   uint32_t elapsed_cnt;
   uint64_t elapsed_ms;

   /* Check that Set_FTM has been called */
   assert(ftm_base);

   /* Verify the delay is achievable without overflowing the FTM */
   assert(ms_delay <= MAX_DELAY);

   FTM_StartTimer(ftm_base, kFTM_FixedClock);
   start_cnt = FTM_GetCurrentTimerCount(ftm_base);

   do {
      elapsed_cnt = FTM_GetCurrentTimerCount(ftm_base) - start_cnt;
      elapsed_ms = COUNT_TO_MSEC(elapsed_cnt, FTM_Clk_Freq);
   } while(elapsed_ms < ms_delay);

   FTM_StopTimer(ftm_base);
   ftm_base->CNT = 0;
}

void MPU6050::Test_Delay(uint16_t ms_delay)
{
   Init_Delay_Timer();
   Delay(ms_delay);
   FTM_Deinit(ftm_base);
}

void MPU6050::Set_FTM(FTM_Type *ftm_base_ptr)
{
   assert(ftm_base_ptr);
   ftm_base = ftm_base_ptr;
}

void MPU6050::Init_I2C(I2C_Type *i2c_base_ptr)
{
   i2c_master_config_t i2c_cfg = {0};

   assert(i2c_base_ptr);
   i2c_base = i2c_base_ptr;

   I2C_MasterGetDefaultConfig(&i2c_cfg);
   I2C_MasterInit(i2c_base, &i2c_cfg, CLOCK_GetFreq(kCLOCK_BusClk));
}

// Configure the motion detection control for low power accelerometer mode
void MPU6050::Low_Power_Accel_Only(void)
{
   // The sensor has a high-pass filter necessary to invoke to allow the sensor motion detection algorithms work properly
   // Motion detection occurs on free-fall (acceleration below a threshold for some time for all axes), motion (acceleration
   // above a threshold for some time on at least one axis), and zero-motion toggle (acceleration on each axis less than a
   // threshold for some time sets this flag, motion above the threshold turns it off). The high-pass filter takes gravity out
   // consideration for these threshold evaluations; otherwise, the flags would be set all the time!

   Init_Delay_Timer();

   uint8_t c = Read_Byte(MPU6050_ADDRESS, PWR_MGMT_1);
   Write_Byte(MPU6050_ADDRESS, PWR_MGMT_1, c & ~0x30); // Clear sleep and cycle bits [5:6]
   Write_Byte(MPU6050_ADDRESS, PWR_MGMT_1, c |  0x30); // Set sleep and cycle bits [5:6] to zero to make sure accelerometer is running

   c = Read_Byte(MPU6050_ADDRESS, PWR_MGMT_2);
   Write_Byte(MPU6050_ADDRESS, PWR_MGMT_2, c & ~0x38); // Clear standby XA, YA, and ZA bits [3:5]
   Write_Byte(MPU6050_ADDRESS, PWR_MGMT_2, c |  0x00); // Set XA, YA, and ZA bits [3:5] to zero to make sure accelerometer is running

   c = Read_Byte(MPU6050_ADDRESS, ACCEL_CONFIG);
   Write_Byte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x07); // Clear high-pass filter bits [2:0]
   // Set high-pass filter to 0) reset (disable), 1) 5 Hz, 2) 2.5 Hz, 3) 1.25 Hz, 4) 0.63 Hz, or 7) Hold
   Write_Byte(MPU6050_ADDRESS, ACCEL_CONFIG,  c | 0x00);  // Set ACCEL_HPF to 0; reset mode disbaling high-pass filter

   c = Read_Byte(MPU6050_ADDRESS, CONFIG);
   Write_Byte(MPU6050_ADDRESS, CONFIG, c & ~0x07); // Clear low-pass filter bits [2:0]
   Write_Byte(MPU6050_ADDRESS, CONFIG, c |  0x00);  // Set DLPD_CFG to 0; 260 Hz bandwidth, 1 kHz rate

   c = Read_Byte(MPU6050_ADDRESS, INT_ENABLE);
   Write_Byte(MPU6050_ADDRESS, INT_ENABLE, c & ~0xFF);  // Clear all interrupts
   Write_Byte(MPU6050_ADDRESS, INT_ENABLE, 0x40);  // Enable motion threshold (bits 5) interrupt only

   // Motion detection interrupt requires the absolute value of any axis to lie above the detection threshold
   // for at least the counter duration
   Write_Byte(MPU6050_ADDRESS, MOT_THR, 0x80); // Set motion detection to 0.256 g; LSB = 2 mg
   Write_Byte(MPU6050_ADDRESS, MOT_DUR, 0x01); // Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate

   Delay(100);  // Add Delay for accumulation of samples

   c = Read_Byte(MPU6050_ADDRESS, ACCEL_CONFIG);
   Write_Byte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x07); // Clear high-pass filter bits [2:0]
   Write_Byte(MPU6050_ADDRESS, ACCEL_CONFIG, c |  0x07);  // Set ACCEL_HPF to 7; hold the initial accleration value as a referance

   c = Read_Byte(MPU6050_ADDRESS, PWR_MGMT_2);
   Write_Byte(MPU6050_ADDRESS, PWR_MGMT_2, c & ~0xC7); // Clear standby XA, YA, and ZA bits [3:5] and LP_WAKE_CTRL bits [6:7]
   Write_Byte(MPU6050_ADDRESS, PWR_MGMT_2, c |  0x47); // Set wakeup frequency to 5 Hz, and disable XG, YG, and ZG gyros (bits [0:2])

   c = Read_Byte(MPU6050_ADDRESS, PWR_MGMT_1);
   Write_Byte(MPU6050_ADDRESS, PWR_MGMT_1, c & ~0x20); // Clear sleep and cycle bit 5
   Write_Byte(MPU6050_ADDRESS, PWR_MGMT_1, c |  0x20); // Set cycle bit 5 to begin low power accelerometer motion interrupts

   FTM_Deinit(ftm_base);
}

void MPU6050::Init(void)
{
   // wake up device-don't need this here if using calibration function below
   //  Write_Byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
   //  Delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

   /* Check that Init_I2C has been called */
   assert(i2c_base);

   // get stable time source
   Write_Byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

   // Configure Gyro and Accelerometer
   // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
   // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
   // Maximum Delay time is 4.9 ms corresponding to just over 200 Hz sample rate
   Write_Byte(MPU6050_ADDRESS, CONFIG, 0x03);

   // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
   Write_Byte(MPU6050_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above

   // Set gyroscope full scale range
   // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
   uint8_t c =  Read_Byte(MPU6050_ADDRESS, GYRO_CONFIG);
   Write_Byte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
   Write_Byte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
   Write_Byte(MPU6050_ADDRESS, GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro

   // Set accelerometer configuration
   c =  Read_Byte(MPU6050_ADDRESS, ACCEL_CONFIG);
   Write_Byte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
   Write_Byte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
   Write_Byte(MPU6050_ADDRESS, ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer

   // Configure Interrupts and Bypass Enable
   // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
   // can join the I2C bus and all can be controlled by the Arduino as master
   Write_Byte(MPU6050_ADDRESS, INT_PIN_CFG, 0x22);
   Write_Byte(MPU6050_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void MPU6050::Calibrate(float * dest1, float * dest2)
{
   uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
   uint16_t ii, packet_count, fifo_count;
   int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

   /* Check that Init_I2C has been called */
   assert(i2c_base);

   Init_Delay_Timer();

   // reset device, reset all registers, clear gyro and accelerometer bias registers
   Write_Byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
   Delay(100);

   // get stable time source
   // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
   Write_Byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);
   Write_Byte(MPU6050_ADDRESS, PWR_MGMT_2, 0x00);
   Delay(200);

   // Configure device for bias calculation
   Write_Byte(MPU6050_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
   Write_Byte(MPU6050_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
   Write_Byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
   Write_Byte(MPU6050_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
   Write_Byte(MPU6050_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
   Write_Byte(MPU6050_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
   Delay(15);

   // Configure MPU6050 gyro and accelerometer for bias calculation
   Write_Byte(MPU6050_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
   Write_Byte(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
   Write_Byte(MPU6050_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
   Write_Byte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

   uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
   uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

   // Configure FIFO to capture accelerometer and gyro data for bias calculation
   Write_Byte(MPU6050_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
   Write_Byte(MPU6050_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
   Delay(80); // accumulate 80 samples in 80 milliseconds = 960 bytes

   // At end of sample accumulation, turn off FIFO sensor read
   Write_Byte(MPU6050_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
   Read_Bytes(MPU6050_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
   fifo_count = ((uint16_t)data[0] << 8) | data[1];
   packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

   for (ii = 0; ii < packet_count; ii++)
   {
      int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
      Read_Bytes(MPU6050_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
      accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
      accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
      accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
      gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
      gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
      gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

      accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
      accel_bias[1] += (int32_t) accel_temp[1];
      accel_bias[2] += (int32_t) accel_temp[2];
      gyro_bias[0]  += (int32_t) gyro_temp[0];
      gyro_bias[1]  += (int32_t) gyro_temp[1];
      gyro_bias[2]  += (int32_t) gyro_temp[2];
   }

   accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
   accel_bias[1] /= (int32_t) packet_count;
   accel_bias[2] /= (int32_t) packet_count;
   gyro_bias[0]  /= (int32_t) packet_count;
   gyro_bias[1]  /= (int32_t) packet_count;
   gyro_bias[2]  /= (int32_t) packet_count;

   if (accel_bias[2] > 0L)
   {
      accel_bias[2] -= (int32_t) accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
   }
   else
   {
      accel_bias[2] += (int32_t) accelsensitivity;
   }

   // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
   data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
   data[1] = (-gyro_bias[0] / 4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
   data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
   data[3] = (-gyro_bias[1] / 4)       & 0xFF;
   data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
   data[5] = (-gyro_bias[2] / 4)       & 0xFF;

   // Push gyro biases to hardware registers
   Write_Byte(MPU6050_ADDRESS, XG_OFFS_USRH, data[0]);// might not be supported in MPU6050
   Write_Byte(MPU6050_ADDRESS, XG_OFFS_USRL, data[1]);
   Write_Byte(MPU6050_ADDRESS, YG_OFFS_USRH, data[2]);
   Write_Byte(MPU6050_ADDRESS, YG_OFFS_USRL, data[3]);
   Write_Byte(MPU6050_ADDRESS, ZG_OFFS_USRH, data[4]);
   Write_Byte(MPU6050_ADDRESS, ZG_OFFS_USRL, data[5]);

   dest1[0] = (float) gyro_bias[0] / (float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
   dest1[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
   dest1[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

   // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
   // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
   // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
   // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
   // the accelerometer biases calculated above must be divided by 8.

   int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
   Read_Bytes(MPU6050_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
   accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
   Read_Bytes(MPU6050_ADDRESS, YA_OFFSET_H, 2, &data[0]);
   accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
   Read_Bytes(MPU6050_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
   accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

   uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
   uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

   for (ii = 0; ii < 3; ii++)
   {
      if (accel_bias_reg[ii] & mask)
      {
         mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
      }
   }

   // Construct total accelerometer bias, including calculated average accelerometer bias from above
   accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
   accel_bias_reg[1] -= (accel_bias[1] / 8);
   accel_bias_reg[2] -= (accel_bias[2] / 8);

   data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
   data[1] = (accel_bias_reg[0])      & 0xFF;
   data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
   data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
   data[3] = (accel_bias_reg[1])      & 0xFF;
   data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
   data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
   data[5] = (accel_bias_reg[2])      & 0xFF;
   data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

   // Push accelerometer biases to hardware registers
   Write_Byte(MPU6050_ADDRESS, XA_OFFSET_H, data[0]); // might not be supported in MPU6050
   Write_Byte(MPU6050_ADDRESS, XA_OFFSET_L_TC, data[1]);
   Write_Byte(MPU6050_ADDRESS, YA_OFFSET_H, data[2]);
   Write_Byte(MPU6050_ADDRESS, YA_OFFSET_L_TC, data[3]);
   Write_Byte(MPU6050_ADDRESS, ZA_OFFSET_H, data[4]);
   Write_Byte(MPU6050_ADDRESS, ZA_OFFSET_L_TC, data[5]);

   // Output scaled accelerometer biases for manual subtraction in the main program
   dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
   dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
   dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;

   FTM_Deinit(ftm_base);
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU6050::Run_Self_Test(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[4];
   uint8_t selfTest[6];
   float factoryTrim[6];

   /* Check that Init_I2C has been called */
   assert(i2c_base);

   Init_Delay_Timer();

   // Configure the accelerometer for self-test
   Write_Byte(MPU6050_ADDRESS, ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
   Write_Byte(MPU6050_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   Delay(250);  // Delay a while to let the device execute the self-test

   rawData[0] = Read_Byte(MPU6050_ADDRESS, SELF_TEST_X); // X-axis self-test results
   rawData[1] = Read_Byte(MPU6050_ADDRESS, SELF_TEST_Y); // Y-axis self-test results
   rawData[2] = Read_Byte(MPU6050_ADDRESS, SELF_TEST_Z); // Z-axis self-test results
   rawData[3] = Read_Byte(MPU6050_ADDRESS, SELF_TEST_A); // Mixed-axis self-test results

   // Extract the acceleration test results first
   selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
   selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 2 ; // YA_TEST result is a five-bit unsigned integer
   selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) ; // ZA_TEST result is a five-bit unsigned integer

   // Extract the gyration test results first
   selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
   selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
   selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer

   // Process results to allow final comparison with factory set values
   factoryTrim[0] = (4096.0 * 0.34) * (pow( (0.92 / 0.34) , (((float)selfTest[0] - 1.0) / 30.0))); // FT[Xa] factory trim calculation
   factoryTrim[1] = (4096.0 * 0.34) * (pow( (0.92 / 0.34) , (((float)selfTest[1] - 1.0) / 30.0))); // FT[Ya] factory trim calculation
   factoryTrim[2] = (4096.0 * 0.34) * (pow( (0.92 / 0.34) , (((float)selfTest[2] - 1.0) / 30.0))); // FT[Za] factory trim calculation
   factoryTrim[3] =  ( 25.0 * 131.0) * (pow( 1.046 , ((float)selfTest[3] - 1.0) ));         // FT[Xg] factory trim calculation
   factoryTrim[4] =  (-25.0 * 131.0) * (pow( 1.046 , ((float)selfTest[4] - 1.0) ));         // FT[Yg] factory trim calculation
   factoryTrim[5] =  ( 25.0 * 131.0) * (pow( 1.046 , ((float)selfTest[5] - 1.0) ));         // FT[Zg] factory trim calculation

   // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
   // To get to percent, must multiply by 100 and subtract result from 100
   for (int i = 0; i < 6; i++)
   {
      destination[i] = 100.0 + 100.0 * ((float)selfTest[i] - factoryTrim[i]) / factoryTrim[i]; // Report percent differences
   }

   FTM_Deinit(ftm_base);
}

float MPU6050::Get_Gyro_Res(void)
{
   switch (Gscale)
   {
      // Possible gyro scales (and their register bit settings) are:
      // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
      // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
      case GFS_250DPS:
         return 250.0 / 32768.0;
         break;
      case GFS_500DPS:
         return 500.0 / 32768.0;
         break;
      case GFS_1000DPS:
         return 1000.0 / 32768.0;
         break;
      case GFS_2000DPS:
         return 2000.0 / 32768.0;
         break;
      default:
         return 0;
         break;
   }
}

float MPU6050::Get_Accel_Res(void)
{
   switch (Ascale)
   {
      // Possible accelerometer scales (and their register bit settings) are:
      // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
      // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
      case AFS_2G:
         return 2.0 / 32768.0;
         break;
      case AFS_4G:
         return 4.0 / 32768.0;
         break;
      case AFS_8G:
         return 8.0 / 32768.0;
         break;
      case AFS_16G:
         return 16.0 / 32768.0;
         break;
      default:
         return 0;
         break;
   }
}

void MPU6050::Read_Accel_Data(int16_t * destination)
{
   uint8_t rawData[6];  // x/y/z accel register data stored here

   /* Check that Init_I2C has been called */
   assert(i2c_base);

   Read_Bytes(MPU6050_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array

   destination[0] = (int16_t)((rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
   destination[1] = (int16_t)((rawData[2] << 8) | rawData[3]) ;
   destination[2] = (int16_t)((rawData[4] << 8) | rawData[5]) ;
}

void MPU6050::Read_Gyro_Data(int16_t * destination)
{
   uint8_t rawData[6];  // x/y/z gyro register data stored here

   /* Check that Init_I2C has been called */
   assert(i2c_base);

   Read_Bytes(MPU6050_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array

   destination[0] = (int16_t)((rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
   destination[1] = (int16_t)((rawData[2] << 8) | rawData[3]) ;
   destination[2] = (int16_t)((rawData[4] << 8) | rawData[5]) ;
}

int16_t MPU6050::Read_Temp_Data(void)
{
   uint8_t rawData[2];  // x/y/z gyro register data stored here

   /* Check that Init_I2C has been called */
   assert(i2c_base);

   Read_Bytes(MPU6050_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array

   return ((int16_t)rawData[0]) << 8 | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

void MPU6050::Write_Byte(uint8_t addr, uint8_t sub_addr, uint8_t data)
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

   status = I2C_MasterTransferBlocking(i2c_base, &masterXfer);

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

uint8_t MPU6050::Read_Byte(uint8_t addr, uint8_t sub_addr)
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

   status = I2C_MasterTransferBlocking(i2c_base, &master_xfer);

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

void MPU6050::Read_Bytes(uint8_t addr, uint8_t sub_addr, uint8_t count, uint8_t * dest)
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

   status = I2C_MasterTransferBlocking(i2c_base, &master_xfer);

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

