/*
 * logging.h
 *
 *  Created on: Jun 16, 2019
 *      Author: Devin
 */

#ifndef LOGGING_H_
#define LOGGING_H_

#include "wheel_speeds.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

extern SemaphoreHandle_t File_Access_Semaphore;

extern void SD_Card_Init_Task(void *pvParameters);
extern void Data_Logging_Task(void *pvParameters);
extern void Open_Log_File(void);
extern void Close_Log_File(void);

#endif /* LOGGING_H_ */
