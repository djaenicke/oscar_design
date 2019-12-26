#include <stdint.h>

#include "lwip/ip_addr.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"


typedef enum {
   UDP_CLIENT_SUCCESS = 1,
   UDP_CLIENT_FAILURE = 2
} UdpClientRetStatus_T;

class UdpClient
{
private:
   struct udp_pcb * control_block;
   ip_addr_t remote_ip;
   uint16_t remote_port = 0;
   pbuf rx_pbuf;
   pbuf tx_pbuf;

public:
   UdpClientRetStatus_T Init(void);
   UdpClientRetStatus_T Disconnect(void);
   UdpClientRetStatus_T Connect(void);
   UdpClientRetStatus_T Set_Remote_Ip(ip_addr_t * ip);
   UdpClientRetStatus_T Set_Remote_Port(uint16_t port);
   UdpClientRetStatus_T Send_Datagram(char *src_buf, uint16_t len);
   UdpClientRetStatus_T Read_Datagram(char *dest_buf, uint16_t max_len);
};
