#include <i2c_mpu6050.h>
#include <mpu6050_registers.h>

typedef enum
{
   X=0,
   Y,
   Z,
   NUM_DIMS
} Dimensions_T;

typedef struct {
   float accel[NUM_DIMS];
   float gyro[NUM_DIMS];
} Meas_Biases_T;

typedef struct {
   float accel;
   float gyro;
} Meas_Scalings_T;

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

class MPU6050:I2C_MPU6050
{
private:
   Meas_Biases_T biases;
   Meas_Scalings_T scalings;

   bool Test_Basic_I2C(void);
   void Calibrate(void);
   void Run_Self_Test(void);
   float Get_Gyro_Res(void);
   float Get_Accel_Res(void);

public:
   void Init(void);
   void Read_Accel_Data(Accel_Data_T * destination);
   void Read_Gyro_Data(Gyro_Data_T * destination);
   void Low_Power_Accel_Only(void);
   float Read_Die_Temp(void);
};
