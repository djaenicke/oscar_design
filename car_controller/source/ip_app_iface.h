/*
 *  ip_app_iface.h
 *
 *  Created on: Mar 24, 2019
 *      Author: Devin
 */

#ifndef IP_APP_IFACE_H_
#define IP_APP_IFACE_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "lwip/opt.h"

#include "lwip/dhcp.h"
#include "lwip/ip_addr.h"
#include "lwip/netifapi.h"
#include "lwip/prot/dhcp.h"
#include "lwip/tcpip.h"
#include "lwip/sys.h"
#include "enet_ethernetif.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
* Prototypes
******************************************************************************/
extern void Init_Network_If(struct netif * net_if);
extern void Print_DHCP_State(struct netif *netif);

#endif /* IP_APP_IFACE_H_ */
