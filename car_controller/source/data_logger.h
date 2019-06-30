/*
 * logging.h
 *
 *  Created on: Jun 16, 2019
 *      Author: Devin
 */

#ifndef DATA_LOGGER_H_
#define DATA_LOGGER_H_

#include "wheel_speeds.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

extern void SD_Card_Init_Task(void *pvParameters);
extern void Log_MC_Stream_Task(void *pvParameters);
extern void Open_Log_File(void);
extern void Close_Log_File(void);

#endif /* DATA_LOGGER_H_ */
