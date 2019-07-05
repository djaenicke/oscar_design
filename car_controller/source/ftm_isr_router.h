/*
 * ftm_isr_router.h
 *
 *  Created on: Jul 4, 2019
 *      Author: Devin
 */

#ifndef FTM_ISR_ROUTER_H_
#define FTM_ISR_ROUTER_H_

#define NUM_FTMS 4

typedef void(*FTM_ISR_Fnc_Ptr_T)(uint8_t);

extern void Reroute_FTM_ISR(uint8_t ftm_num, FTM_ISR_Fnc_Ptr_T func_ptr);

#endif /* FTM_ISR_ROUTER_H_ */
