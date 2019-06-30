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
   Wheel_Speeds_T raw_speeds;
   Wheel_Speeds_T filt_speeds;
   float meas_vbatt;
   float max_vbatt;
   float max_speed;
   float min_speed;
   float r_speed_sp;
   float l_speed_sp;
   float r_speed_fb;
   float l_speed_fb;
   float r_error;
   float l_error;
   float r_integral;
   float l_integral;
   float u_r;
   float u_l;
   uint8_t u_r_dc;
   uint8_t u_l_dc;
   uint32_t end_pattern;
} Motor_Controls_Stream_T;

extern Stream_Info_T MC_Stream;

extern void Create_Streams(void);

#endif /* LOGGING_STREAMS_H_ */
