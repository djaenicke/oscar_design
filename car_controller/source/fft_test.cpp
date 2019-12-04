/*
 * fft_test.cpp
 *
 *  Created on: Nov 30, 2019
 *      Author: Devin
 */

#include "fft_test.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "arm_fft_bin_data.h"
#include "assert.h"
#include "fsl_debug_console.h"
#include "fsl_ftm.h"

#if RUN_FFT_TEST
#define MIN_IDX    (13)
#define MAX_IDX    (302)
#define BIN_WIDTH  (0.0799f) /* FS/NUM_SAMPLES/192*2*PI */
#define CLK_PERIOD (0.00002048f)

extern float32_t FFT_Buffer[NUM_SAMPLES]; /* Input and Output Buffer */

arm_rfft_fast_instance_f32  RFFT_Instance;
uint32_t IFFT_Flag = 0;

void Run_FFT_Test(void)
{
   ftm_config_t ftmInfo;
   arm_status   status;
   float        max_value, wheel_speed, dt;
   uint32_t     max_idx;
   uint16_t     start_cnt;

   /* Initialize FTM module 1 for timing the FFT execution times */
   FTM_GetDefaultConfig(&ftmInfo);
   ftmInfo.prescale = kFTM_Prescale_Divide_32;
   FTM_Init(FTM1, &ftmInfo);
   FTM_SetTimerPeriod(FTM1, UINT16_MAX);
   FTM_StartTimer(FTM1, kFTM_FixedClock);
   /* Finish FTM init */

   status  = arm_rfft_fast_init_f32(&RFFT_Instance, NUM_SAMPLES);

   if (ARM_MATH_SUCCESS == status)
   {
      start_cnt = (uint16_t) FTM1->CNT;

      arm_rfft_fast_f32(&RFFT_Instance, FFT_Buffer, FFT_Buffer, IFFT_Flag);
      arm_cmplx_mag_f32(FFT_Buffer, FFT_Buffer, NUM_SAMPLES/2);
      arm_max_f32(&(FFT_Buffer[MIN_IDX]), MAX_IDX-MIN_IDX, &max_value, &max_idx);

      wheel_speed = (max_idx + MIN_IDX) * BIN_WIDTH;
      dt = (FTM1->CNT - start_cnt) * CLK_PERIOD;

      PRINTF("FFT execution time = %d (us)\n\r", (uint16_t)(dt*1000000));
   }
   else
   {
      assert(false);
   }

   FTM_Deinit(FTM1);
}
#endif

