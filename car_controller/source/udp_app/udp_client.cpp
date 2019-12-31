#include <string.h>
#include "udp_client.h"

UdpClientRetStatus_T UdpClient::Init(struct netif *netif, uint16_t client_port)
{
   UdpClientRetStatus_T ret_val = UDP_CLIENT_FAILURE;
   err_t lwip_error_code;

   if (netif)
   {
      net_if = netif;
      control_block = udp_new();

      if (control_block)
      {
         udp_recv(control_block, Rx_Callback, this);
         udp_bind_netif(control_block, netif);
         lwip_error_code = udp_bind(control_block, (const ip_addr_t *)&(netif->ip_addr), client_port);

         if (ERR_OK == lwip_error_code)
         {
            initialized = true;
            ret_val = UDP_CLIENT_SUCCESS;
            connected = true;
         }
      }
   }

   return ret_val;
}

bool UdpClient::Is_Initialized(void)
{
   return initialized;
}

UdpClientRetStatus_T UdpClient::Set_Remote_Ip(const char *cp)
{
   UdpClientRetStatus_T ret_val = UDP_CLIENT_FAILURE;

   if (cp)
   {
      remote_ip.addr = ipaddr_addr(cp);

      if (IPADDR_NONE != remote_ip.addr)
      {
         ret_val = UDP_CLIENT_SUCCESS;
      }
   }

   return ret_val;
}

UdpClientRetStatus_T UdpClient::Set_Remote_Port(uint16_t port)
{
   remote_port = port;
   return UDP_CLIENT_SUCCESS;
}

UdpClientRetStatus_T UdpClient::Connect(void)
{
   err_t lwip_error_code;
   UdpClientRetStatus_T ret_val = UDP_CLIENT_FAILURE;

   lwip_error_code = udp_connect(control_block, (const ip_addr_t *)&remote_ip, remote_port);
   if (ERR_OK == lwip_error_code)
   {
      ret_val = UDP_CLIENT_SUCCESS;
      connected = true;
   }

   return ret_val;
}

bool UdpClient::Is_Connected(void)
{
   return connected;
}

UdpClientRetStatus_T UdpClient::Disconnect(void)
{
   udp_disconnect(control_block);
   connected = false;
   return UDP_CLIENT_SUCCESS;
}

UdpClientRetStatus_T UdpClient::Send_Datagram(uint8_t *src_buf, uint16_t len)
{
   err_t lwip_error_code;
   UdpClientRetStatus_T ret_val;
   pbuf * tx_pbuf;

   tx_pbuf = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);

   if (tx_pbuf)
   {
      tx_pbuf->payload = src_buf;
      lwip_error_code = udp_send(control_block, tx_pbuf);
   }
   else
   {
      lwip_error_code = ERR_MEM;
   }

   if (ERR_OK == lwip_error_code)
   {
      ret_val = UDP_CLIENT_SUCCESS;
      pbuf_free(tx_pbuf);
   }
   else
   {
      ret_val = UDP_CLIENT_FAILURE;
   }

   return ret_val;
}

uint16_t UdpClient::Rx_Bytes_Available(void)
{
   return(rx_bytes_avail);
}

UdpClientRetStatus_T UdpClient::Read_Datagram(uint8_t *dest_buf, uint16_t max_len)
{
   UdpClientRetStatus_T ret_val = UDP_CLIENT_FAILURE;
   working_rx_pbuf = orignal_rx_pbuf; /* Reset just in case */

   if (orignal_rx_pbuf->tot_len <= max_len) /* Is dest_buf large enough? */
   {
      do
      {
         memcpy(&dest_buf, working_rx_pbuf->payload, working_rx_pbuf->len);
         dest_buf += working_rx_pbuf->len;
         working_rx_pbuf = working_rx_pbuf->next;
      } while (working_rx_pbuf);

      /* Free the original pbuf */
      pbuf_free(orignal_rx_pbuf);
      rx_bytes_avail = 0;
      ret_val = UDP_CLIENT_SUCCESS;
   }

   return ret_val;
}

char UdpClient::Read_Byte(void)
{
   char c;

   if (rx_bytes_avail)
   {
      c = *((char *)working_rx_pbuf->payload + pbuf_offset);
      pbuf_offset++;
      rx_bytes_avail--;

      if (pbuf_offset > ((working_rx_pbuf->len)-1))
      {
         pbuf_offset = 0;
         working_rx_pbuf = working_rx_pbuf->next;
      }

      if (0 == rx_bytes_avail)
      {
         pbuf_free(orignal_rx_pbuf);
      }
   }

   return(c);
}

void UdpClient::Rx_Callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
   UdpClient * this_ptr = (UdpClient *) arg;

   if (0 == this_ptr->rx_bytes_avail)
   {
      this_ptr->orignal_rx_pbuf = p;
      this_ptr->working_rx_pbuf = p;
      this_ptr->rx_bytes_avail = p->tot_len;
      this_ptr->pbuf_offset = 0;
   }
   else
   {
      this_ptr->rx_datagrams_dropped++;
   }
}
