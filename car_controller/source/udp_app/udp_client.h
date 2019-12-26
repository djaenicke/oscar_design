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
   struct netif * net_if;

   ip_addr_t remote_ip;
   uint16_t remote_port = 0;

   bool initialized = false;
   bool connected = false;

   struct pbuf *orignal_rx_pbuf = NULL;
   struct pbuf *working_rx_pbuf = NULL;
   uint16_t rx_bytes_avail = 0;
   uint16_t pbuf_offset = 0;
   uint16_t rx_datagrams_dropped = 0;

   static void Rx_Callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

public:
   UdpClientRetStatus_T Init(struct netif *netif, uint16_t client_port);
   bool Is_Initialized(void);
   UdpClientRetStatus_T Disconnect(void);
   UdpClientRetStatus_T Connect(void);
   bool Is_Connected(void);
   UdpClientRetStatus_T Set_Remote_Ip(const char *cp);
   UdpClientRetStatus_T Set_Remote_Port(uint16_t port);
   UdpClientRetStatus_T Send_Datagram(char *src_buf, uint16_t len);
   UdpClientRetStatus_T Read_Datagram(char *dest_buf, uint16_t max_len);
   uint16_t Rx_Bytes_Available(void);
   char Read_Byte(void);
};
