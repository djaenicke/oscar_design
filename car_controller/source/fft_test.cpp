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

#if RUN_FFT_TEST
#define MIN_IDX   (52)
#define BIN_WIDTH (0.02f) /* FS/NUM_SAMPLES/192*2*PI */

extern float32_t FFT_Buffer[NUM_SAMPLES]; /* Input and Output Buffer */

uint32_t IFFT_Flag = 0;
uint32_t refIndex = 701, testIndex = 0;
arm_rfft_fast_instance_f32  RFFT_Instance;

void Run_FFT_Test(void)
{
   arm_status status;
   float      max_value;
   uint32_t   max_idx;
   float      wheel_speed;

   status = ARM_MATH_SUCCESS;

   status = arm_rfft_fast_init_f32(&RFFT_Instance, NUM_SAMPLES);

   if (ARM_MATH_SUCCESS == status)
   {
      arm_rfft_fast_f32(&RFFT_Instance, FFT_Buffer, FFT_Buffer, IFFT_Flag);
      arm_cmplx_mag_f32(FFT_Buffer, FFT_Buffer, NUM_SAMPLES/2);

      /* Calculates maxValue and returns corresponding BIN value */
      arm_max_f32(&(FFT_Buffer[MIN_IDX]), (NUM_SAMPLES/2)-MIN_IDX, &max_value, &max_idx);

      wheel_speed = (max_idx + MIN_IDX) * BIN_WIDTH;

      for (int i=0; i<NUM_SAMPLES/2; i++)
      {
         PRINTF("%d\n\r", (uint16_t)(FFT_Buffer[i]*1000));
      }
   }
   else
   {
      assert(false);
   }
}
#endif
