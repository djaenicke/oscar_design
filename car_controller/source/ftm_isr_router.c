/*
 * ftm_isr_router.c
 *
 *  Created on: Jul 4, 2019
 *      Author: Devin
 */
#include <stdint.h>
#include <string.h>

#include "ftm_isr_router.h"
#include "assert.h"

static FTM_ISR_Fnc_Ptr_T FTM_ISR_Table[NUM_FTMS];

void Reroute_FTM_ISR(uint8_t ftm_num, FTM_ISR_Fnc_Ptr_T func_ptr)
{
   /* Check for null function pointer */
   assert(func_ptr);

   if (ftm_num < NUM_FTMS)
   {
      if (NULL == FTM_ISR_Table[ftm_num])
      {
         FTM_ISR_Table[ftm_num] = func_ptr;
      }
      else
      {
         /* FTM ISR already rerouted!!! */
         assert(0);
      }
   }
   else
   {
      assert(0);
   }
}


void FTM0_IRQHandler(void)
{
   if (NULL != FTM_ISR_Table[0])
   {
      FTM_ISR_Table[0]((uint8_t)0);
   }
   else
   {
      assert(0);
   }
}

void FTM1_IRQHandler(void)
{
   if (NULL != FTM_ISR_Table[1])
   {
      FTM_ISR_Table[1]((uint8_t)1);
   }
   else
   {
      assert(0);
   }
}

void FTM2_IRQHandler(void)
{
   if (NULL != FTM_ISR_Table[2])
   {
      FTM_ISR_Table[2]((uint8_t)2);
   }
   else
   {
      assert(0);
   }
}

void FTM3_IRQHandler(void)
{
   if (NULL != FTM_ISR_Table[3])
   {
      FTM_ISR_Table[3]((uint8_t)3);
   }
   else
   {
      assert(0);
   }
}
