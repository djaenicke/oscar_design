/*
 * port_isr_router.c
 *
 *  Created on: Jul 5, 2019
 *      Author: Devin
 */
#include <stdint.h>
#include <string.h>

#include "MK64F12.h"
#include "port_isr_router.h"
#include "assert.h"

static Port_ISR_Fnc_Ptr_T Port_ISR_Table[NUM_PORTS];

void Reroute_Port_ISR(uint32_t port_addr, Port_ISR_Fnc_Ptr_T func_ptr)
{
   uint8_t port_num;

   /* Check for null function pointer */
   assert(func_ptr);

   switch(port_addr)
   {
      case PORTA_BASE:
         port_num = 0;
         break;
      case PORTB_BASE:
         port_num = 1;
         break;
      case PORTC_BASE:
         port_num = 2;
         break;
      case PORTD_BASE:
         port_num = 3;
         break;
      case PORTE_BASE:
         port_num = 4;
         break;
      default:
         assert(0);
   }

   if (NULL == Port_ISR_Table[port_num])
   {
      Port_ISR_Table[port_num] = func_ptr;
   }
   else
   {
      /* Port ISR has already been rerouted!!! */
      assert(0);
   }
}

#if (REROUTE_PORTA)
void PORTA_IRQHandler(void)
{
   if (NULL != Port_ISR_Table[0])
   {
      Port_ISR_Table[0]((uint32_t)PORTA_BASE);
   }
   else
   {
      assert(0);
   }
}
#endif

#if (REROUTE_PORTB)
void PORTB_IRQHandler(void)
{
   if (NULL != Port_ISR_Table[1])
   {
      Port_ISR_Table[1]((uint32_t)PORTB_BASE);
   }
   else
   {
      assert(0);
   }
}
#endif

#if (REROUTE_PORTC)
void PORTC_IRQHandler(void)
{
   if (NULL != Port_ISR_Table[2])
   {
      Port_ISR_Table[2]((uint32_t)PORTC_BASE);
   }
   else
   {
      assert(0);
   }
}
#endif

#if (REROUTE_PORTD)
void PORTD_IRQHandler(void)
{
   if (NULL != Port_ISR_Table[3])
   {
      Port_ISR_Table[3]((uint32_t)PORTD_BASE);
   }
   else
   {
      assert(0);
   }
}
#endif

#if (REROUTE_PORTE)
void PORTE_IRQHandler(void)
{
   if (NULL != Port_ISR_Table[4])
   {
      Port_ISR_Table[4]((uint32_t)PORTE_BASE);
   }
   else
   {
      assert(0);
   }
}
#endif

