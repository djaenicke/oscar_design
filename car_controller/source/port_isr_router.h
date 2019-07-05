/*
 * port_isr_router.h
 *
 *  Created on: Jul 5, 2019
 *      Author: Devin
 */

#ifndef PORT_ISR_ROUTER_H_
#define PORT_ISR_ROUTER_H_

#define REROUTE_PORTA (1)
#define REROUTE_PORTB (0)
#define REROUTE_PORTC (0)
#define REROUTE_PORTD (1)
#define REROUTE_PORTE (0)

#define NUM_PORTS 5

typedef void(*Port_ISR_Fnc_Ptr_T)(uint32_t);

extern void Reroute_Port_ISR(uint32_t port_addr, Port_ISR_Fnc_Ptr_T func_ptr);

#endif /* PORTS_ISR_ROUTER_H_ */
