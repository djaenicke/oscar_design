/*
 * logging_streams.cpp
 *
 *  Created on: Jun 30, 2019
 *      Author: Devin
 */

#include "FreeRTOS.h"
#include "logging_streams.h"
#include "assert.h"

/* Stream declarations */
Stream_Info_T MC_Stream = {0};

void Create_Streams(void)
{
   /* Motor Controls Stream */
   MC_Stream.size = sizeof(Motor_Controls_Stream_T);
   MC_Stream.trigger_level = sizeof(Motor_Controls_Stream_T);
   MC_Stream.handle = xStreamBufferCreate(MC_Stream.size, MC_Stream.trigger_level);
   assert(NULL != MC_Stream.handle);
}


