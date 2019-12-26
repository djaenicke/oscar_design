#include <string.h>
#include "udp_client.h"

UdpClientRetStatus_T UdpClient::Init(void)
{
   UdpClientRetStatus_T ret_val;

   control_block = udp_new();

   if (control_block)
   {
      ret_val = UDP_CLIENT_SUCCESS;
   }
   else
   {
      ret_val = UDP_CLIENT_FAILURE;
   }

   return ret_val;
}

UdpClientRetStatus_T UdpClient::Set_Remote_Ip(ip_addr_t * ip)
{
   UdpClientRetStatus_T ret_val;

   if (ip)
   {
      memcpy(&remote_ip, ip, sizeof(remote_ip));
      ret_val = UDP_CLIENT_SUCCESS;
   }
   else
   {
      ret_val = UDP_CLIENT_FAILURE;
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
   UdpClientRetStatus_T ret_val;

   lwip_error_code = udp_connect(control_block, (const ip_addr_t *)&remote_ip, remote_port);

   if (ERR_OK == lwip_error_code)
   {
      ret_val = UDP_CLIENT_SUCCESS;
   }
   else
   {
      ret_val = UDP_CLIENT_FAILURE;
   }

   return ret_val;
}

UdpClientRetStatus_T UdpClient::Disconnect(void)
{
   udp_disconnect(control_block);
   return UDP_CLIENT_SUCCESS;
}

UdpClientRetStatus_T UdpClient::Send_Datagram(char *src_buf, uint16_t len)
{
   err_t lwip_error_code;
   UdpClientRetStatus_T ret_val;

   tx_pbuf.next = 0;
   tx_pbuf.payload = src_buf;
   tx_pbuf.len = len;
   tx_pbuf.tot_len = len;

   lwip_error_code = udp_send(control_block, &tx_pbuf);

   if (ERR_OK == lwip_error_code)
   {
      ret_val = UDP_CLIENT_SUCCESS;
   }
   else
   {
      ret_val = UDP_CLIENT_FAILURE;
   }

   return ret_val;
}
