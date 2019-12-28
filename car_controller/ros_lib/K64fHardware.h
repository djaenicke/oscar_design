/*
 * K64fHardware
 *
 *  Created on: Dec 27, 2019
 *      Author: Devin
 */

#ifndef ROS_K64F_HARDWARE_H_
#define ROS_K64F_HARDWARE_H_

#include "udp_client.h"
#include "ip_app_iface.h"
#include "task.h"

class K64fHardware
{
public:
   /* Any initialization code necessary to use the serial port */
	void init()
	{
		ROS_UDP.Init(Get_Netif(), 5000);
		ROS_UDP.Set_Remote_Ip("192.168.1.4");
		ROS_UDP.Set_Remote_Port(5000);
		ROS_UDP.Connect();
	}

	/* Read a byte from the serial port. -1 = failure */
	int read()
	{
	   int ret_val = -1;

	   if (ROS_UDP.Rx_Bytes_Available())
	   {
	      ret_val = (int) ROS_UDP.Read_Byte();
	   }

	   return (ret_val);
	}

	/* Write data to the connection to ROS */
	void write(uint8_t* data, int length)
	{
		ROS_UDP.Send_Datagram(data, (uint16_t) length);
	}

	/* Returns milliseconds since start of program */
	unsigned long time()
	{
		return xTaskGetTickCount();
	}

protected:
	UdpClient ROS_UDP;
};


#endif /* ROS_K64F_HARDWARE_H_ */
