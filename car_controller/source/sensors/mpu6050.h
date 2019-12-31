#include <i2c_mpu6050.h>
#include <mpu6050_registers.h>

typedef struct {
   float accel[3];
   float gyro[3];
} MPU6050_Meas_Biases_T;

typedef struct {
   float accel;
   float gyro;
} MPU6050_Meas_Scalings_T;

typedef struct {
   float ax;
   float ay;
   float az;
} Accel_Data_T;

typedef struct {
   float gx;
   float gy;
   float gz;
} Gyro_Data_T;

typedef enum
{
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
} MPU6050_Ascale_T;

typedef enum
{
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
} MPU6050_Gscale_T;

class MPU6050:I2C_MPU6050
{
private:
   MPU6050_Ascale_T a_scale;
   MPU6050_Gscale_T g_scale;
   MPU6050_Meas_Biases_T biases;
   MPU6050_Meas_Scalings_T scalings;

   bool Test_Basic_I2C(void);
   void Calibrate(void);
   void Run_Self_Test(void);
   void Set_Gyro_Res(MPU6050_Gscale_T gscale);
   void Set_Accel_Res(MPU6050_Ascale_T ascale);

public:
   void Init(MPU6050_Ascale_T ascale, MPU6050_Gscale_T gscale);
   void Read_Accel_Data(Accel_Data_T * destination);
   void Read_Gyro_Data(Gyro_Data_T * destination);
   void Low_Power_Accel_Only(void);
   float Read_Die_Temp(void);
};
