/*
 * bluetooth_control.h
 *
 *  Created on: Jun 9, 2019
 *      Author: Devin
 */

#ifndef BLUETOOTH_CONTROL_H_
#define BLUETOOTH_CONTROL_H_

extern void Bluetooth_Serial_Open(void);
extern void Bluetooth_Serial_Close(void);
extern void Bluetooth_Cmd_Task(void *pvParameters);

#endif /* BLUETOOTH_CONTROL_H_ */
