/*
 * logging_streams.h
 *
 *  Created on: Jun 30, 2019
 *      Author: Devin
 */

#ifndef LOGGING_STREAMS_H_
#define LOGGING_STREAMS_H_

#include <stdint.h>
#include "wheel_speeds.h"
#include "stream_buffer.h"

#define END_PATTERN (0xFFFFFFFF)

typedef struct
{
   StreamBufferHandle_t handle;
   size_t size;
   size_t trigger_level;
} Stream_Info_T;

typedef struct
{
   uint32_t cnt;
   Wheel_Speeds_T wheel_ang_v;
   float meas_vbatt;
   float max_vbatt;
   float r_ang_v_sp;
   float l_ang_v_sp;
   uint8_t u_r_dc;
   uint8_t u_l_dc;
   uint32_t end_pattern;
} Motor_Controls_Stream_T;

extern Stream_Info_T MC_Stream;

extern void Create_Streams(void);

#endif /* LOGGING_STREAMS_H_ */
