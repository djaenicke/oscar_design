/*
 * wheel_speeds_fft.cpp
 *
 *  Created on: Dec 1, 2019
 *      Author: Devin
 */
#include <stdint.h>
#include <string.h>

#include "arm_math.h"
#include "arm_const_structs.h"
#include "arm_fft_bin_data.h"

#include "fsl_ftm.h"
#include "fsl_port.h"
#include "assert.h"

#include "wheel_speeds_fft.h"
#include "io_abstraction.h"
#include "interrupt_prios.h"
extern "C"
{
#include "ftm_isr_router.h"
}

#define fs 2500                 /* Sample frequency (Hz)   */
#define FFT_SIZE 1024U          /* FFT input Size          */
#define SAMPLES_PER_WINDOW 62   /* Wheel speed samples per window */

#define BYTES_PER_SAMPLE_WINDOW ((size_t) 248)  /* SAMPLES_PER_WINDOW * sizeof(float) */
#define BYTES_PER_FFT           ((size_t) 4096) /* FFT_SIZE * sizeof(float) */

#define MIN_IDX    (13)
#define MAX_IDX    (302)
#define BIN_WIDTH  (0.0799f) /* fs/FFT_SIZE/192*2*PI */

static bool Timer_Started = false;

static volatile uint8_t Samples_Taken = 0;
static volatile float ISR_Samples[NUM_WHEELS][SAMPLES_PER_WINDOW] = {{0}};

static float FFT_Buffer[FFT_SIZE]; /* Input and Output Buffer */
static arm_rfft_fast_instance_f32 RFFT_Instance;
static uint32_t IFFT_Flag = 0;

extern "C"
{
static void FFT_Sample_FTM_IRQHandler(uint8_t ftm_num);
}

void Init_Wheel_Speeds_FFT(void)
{
   ftm_config_t ftmInfo;

   FTM_GetDefaultConfig(&ftmInfo);

   /* Initialize FTM module */
   FTM_Init(FTM1, &ftmInfo);
   FTM_SetTimerPeriod(FTM1, (1.0/fs)/(1.0f/CLOCK_GetFreq(kCLOCK_BusClk)));
   Reroute_FTM_ISR((uint8_t)1, &FFT_Sample_FTM_IRQHandler);
   EnableIRQ(FTM1_IRQn);
   FTM_EnableInterrupts(FTM1, kFTM_TimeOverflowInterruptEnable);

   /* Initialize FFT */
   if (ARM_MATH_SUCCESS != arm_rfft_fast_init_f32(&RFFT_Instance, FFT_SIZE))
   {
      assert(false);
   }
}

void Get_Wheel_Speeds_FFT(Wheel_Speeds_T * speeds)
{
   float    max_value;
   uint32_t max_idx;
   float    wheel_speed = 0.0f;

   assert(speeds);

   if (!Timer_Started)
   {
      FTM_StartTimer(FTM1, kFTM_SystemClock);
      Timer_Started = true;
   }
   else
   {
      FTM_DisableInterrupts(FTM1, kFTM_TimeOverflowInterruptEnable);
      Samples_Taken = 0;

      /* Run the FFTs */
      for (uint8_t i=0; i < NUM_WHEELS; i++)
      {
         memset(FFT_Buffer, 0, BYTES_PER_FFT);
         memcpy(FFT_Buffer, (const void *)&(ISR_Samples[i]), BYTES_PER_SAMPLE_WINDOW);

         arm_rfft_fast_f32(&RFFT_Instance, FFT_Buffer, FFT_Buffer, IFFT_Flag);
         arm_cmplx_mag_f32(FFT_Buffer, FFT_Buffer, FFT_SIZE/2);
         arm_max_f32(&(FFT_Buffer[MIN_IDX]), MAX_IDX-MIN_IDX, &max_value, &max_idx);

         if (max_idx)
         {
            wheel_speed = (max_idx + MIN_IDX) * BIN_WIDTH;
         }

         if ((uint8_t)R == i)
         {
            speeds->r = wheel_speed;
         }
         else
         {
            speeds->l = wheel_speed;
         }
      }

      FTM_EnableInterrupts(FTM1, kFTM_TimeOverflowInterruptEnable);
   }
}

extern "C"
{
void FFT_Sample_FTM_IRQHandler(uint8_t ftm_num)
{
   FTM_StopTimer(FTM1);
   FTM1->CNT = 0;
   FTM_StartTimer(FTM1, kFTM_SystemClock);

   if (Samples_Taken < SAMPLES_PER_WINDOW)
   {
      ISR_Samples[R][Samples_Taken] = (float) Read_GPIO(R_SPEED_SENSOR);
      ISR_Samples[L][Samples_Taken] = (float) Read_GPIO(L_SPEED_SENSOR);
      Samples_Taken++;
   }

   /* Clear interrupt flag.*/
    FTM_ClearStatusFlags(FTM1, kFTM_TimeOverflowFlag);
    __DSB();
}
}

