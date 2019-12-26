/*
 * battery_monitor.cpp
 *
 *  Created on: Jun 10, 2019
 *      Author: Devin
 */

#include "battery_monitor.h"
#include "fsl_adc16.h"

#define ADC_MEAS_CHANNEL 12U
#define VOLTS_PER_COUNT  (3.3f/4095)
#define R1               (4.65f)
#define R2               (2.161f)
#define SCALING          ((R1+R2)/R2)

void Init_Battery_Monitor(void)
{
   adc16_config_t adc16_cfg;
   adc16_channel_config_t adc16ChannelConfigStruct;

   ADC16_GetDefaultConfig(&adc16_cfg);

   adc16_cfg.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
   adc16_cfg.enableContinuousConversion = true;


   ADC16_Init(ADC0, &adc16_cfg);
   ADC16_EnableHardwareTrigger(ADC0, false); /* Make sure the software trigger is used. */

   adc16ChannelConfigStruct.channelNumber = ADC_MEAS_CHANNEL;
   adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;

   ADC16_SetChannelConfig(ADC0, 0, &adc16ChannelConfigStruct);
}

float Read_Battery_Voltage(void)
{
   uint32_t measurement;
   float vbatt;

   measurement = ADC16_GetChannelConversionValue(ADC0, 0);
   vbatt = measurement * VOLTS_PER_COUNT * SCALING;

   return (vbatt);
}

