/*
 * ip_app_iface.c
 *
 *  Created on: Mar 24, 2019
 *      Author: Devin
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "board.h"
#include "ip_app_iface.h"
#include "constants.h"

#if USE_ETHERNET
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* MAC address configuration. */
#define configMAC_ADDR                 \
{                                      \
    0x02, 0x12, 0x13, 0x10, 0x15, 0x12 \
}

/* Address of PHY interface. */
#define EXAMPLE_PHY_ADDRESS BOARD_ENET0_PHY_ADDRESS

/*******************************************************************************
* Variables
******************************************************************************/
static struct netif FSL_NetIf;
static Network_Status_T Network_Status = NOT_CONNECTED;

/*******************************************************************************
 * Function Definitions
 ******************************************************************************/
void Init_Network_If(void)
{
    ip4_addr_t fsl_netif0_ipaddr, fsl_netif0_netmask, fsl_netif0_gw;
    ethernetif_config_t fsl_enet_config0 = {
        .phyAddress = EXAMPLE_PHY_ADDRESS, .clockName = kCLOCK_CoreSysClk, .macAddress = configMAC_ADDR,
    };

    IP4_ADDR(&fsl_netif0_ipaddr, 0U, 0U, 0U, 0U);
    IP4_ADDR(&fsl_netif0_netmask, 0U, 0U, 0U, 0U);
    IP4_ADDR(&fsl_netif0_gw, 0U, 0U, 0U, 0U);

    tcpip_init(NULL, NULL);

    netifapi_netif_add(&FSL_NetIf, &fsl_netif0_ipaddr, &fsl_netif0_netmask, &fsl_netif0_gw, &fsl_enet_config0,
                       ethernetif0_init, tcpip_input);
    netifapi_netif_set_default(&FSL_NetIf);
    netifapi_netif_set_up(&FSL_NetIf);

    netifapi_dhcp_start(&FSL_NetIf);
}

void Print_DHCP_State(void)
{
    struct dhcp *dhcp;
    static uint8_t dhcp_last_state = DHCP_STATE_OFF;

    dhcp = netif_dhcp_data(&FSL_NetIf);

    if (dhcp == NULL)
    {
        dhcp_last_state = DHCP_STATE_OFF;
    }
    else if (dhcp_last_state != dhcp->state)
    {
        dhcp_last_state = dhcp->state;

        PRINTF(" DHCP state       : ");
        switch (dhcp_last_state)
        {
            case DHCP_STATE_OFF:
                PRINTF("OFF\r\n");
                break;
            case DHCP_STATE_REQUESTING:
                PRINTF("REQUESTING\r\n");
                break;
            case DHCP_STATE_INIT:
                PRINTF("INIT\r\n");
                break;
            case DHCP_STATE_REBOOTING:
                PRINTF("REBOOTING\r\n");
                break;
            case DHCP_STATE_REBINDING:
                PRINTF("REBINDING\r\n");
                break;
            case DHCP_STATE_RENEWING:
                PRINTF("RENEWING\r\n");
                break;
            case DHCP_STATE_SELECTING:
                PRINTF("SELECTING\r\n");
                break;
            case DHCP_STATE_INFORMING:
                PRINTF("INFORMING\r\n");
                break;
            case DHCP_STATE_CHECKING:
                PRINTF("CHECKING\r\n");
                break;
            case DHCP_STATE_BOUND:
                PRINTF("BOUND");
                break;
            case DHCP_STATE_BACKING_OFF:
                PRINTF("BACKING_OFF\r\n");
                break;
            default:
                PRINTF("%u", dhcp_last_state);
                assert(0);
                break;
        }

        if (dhcp_last_state == DHCP_STATE_BOUND)
        {
            PRINTF("\r\n");
            PRINTF("\r\n IPv4 Address     : %s\r\n", ipaddr_ntoa(&(FSL_NetIf.ip_addr)));
            PRINTF(" IPv4 Subnet mask : %s\r\n", ipaddr_ntoa(&(FSL_NetIf.netmask)));
            PRINTF(" IPv4 Gateway     : %s\r\n\r\n", ipaddr_ntoa(&(FSL_NetIf.gw)));
        }
    }
}
#endif

struct netif * Get_Netif(void)
{
#if USE_ETHERNET
   return &FSL_NetIf;
#else
   return NULL;
#endif
}

Network_Status_T Get_Network_Status(void)
{
#if USE_ETHERNET
   struct dhcp *dhcp;

   /* Check the network connection status */
   dhcp = netif_dhcp_data(&FSL_NetIf);

   if (netif_is_up(&FSL_NetIf) && DHCP_STATE_BOUND == dhcp->state)
   {
      Network_Status = CONNECTED;
   }
   else
   {
      Network_Status = NOT_CONNECTED;
   }

   return (Network_Status);
#else
   return (NOT_CONNECTED);
#endif
}
