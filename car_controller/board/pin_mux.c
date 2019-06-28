/*
 * Copyright 2017-2018 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v4.0
* BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********/

/**
 * @file    pin_mux.c
 * @brief   Board pins file.
 */
 
/* This is a template for board specific configuration created by MCUXpresso IDE Project Wizard.*/

#include "pin_mux.h"
#include "fsl_port.h"
#include "io_abstraction.h"
#include "interrupt_prios.h"

/**
 * @brief Set up and initialize all required blocks and functions related to the board hardware.
 */
void BOARD_InitBootPins(void)
{
   uint8_t i;

   /* Enable Port Clock Gate Controls */
   CLOCK_EnableClock(kCLOCK_PortA);
   CLOCK_EnableClock(kCLOCK_PortB);
   CLOCK_EnableClock(kCLOCK_PortC);
   CLOCK_EnableClock(kCLOCK_PortD);
   CLOCK_EnableClock(kCLOCK_PortE);


   for (i=0; i<NUM_IO; i++)
   {
       PORT_SetPinMux(Pin_Cfgs[i].pbase, Pin_Cfgs[i].pin, Pin_Cfgs[i].mux);
   }

   const port_pin_config_t porte0_pin1_config = {
     kPORT_PullUp,                                            /* Internal pull-up resistor is enabled */
     kPORT_FastSlewRate,                                      /* Fast slew rate is configured */
     kPORT_PassiveFilterDisable,                              /* Passive filter is disabled */
     kPORT_OpenDrainDisable,                                  /* Open drain is disabled */
     kPORT_HighDriveStrength,                                 /* High drive strength is configured */
     kPORT_MuxAlt4,                                           /* Pin is configured as SDHC0_D1 */
     kPORT_UnlockRegister                                     /* Pin Control Register fields [15:0] are not locked */
   };
   PORT_SetPinConfig(PORTE, 0, &porte0_pin1_config);   /* PORTE0 (pin 1) is configured as SDHC0_D1 */
   const port_pin_config_t porte1_pin2_config = {
     kPORT_PullUp,                                            /* Internal pull-up resistor is enabled */
     kPORT_FastSlewRate,                                      /* Fast slew rate is configured */
     kPORT_PassiveFilterDisable,                              /* Passive filter is disabled */
     kPORT_OpenDrainDisable,                                  /* Open drain is disabled */
     kPORT_HighDriveStrength,                                 /* High drive strength is configured */
     kPORT_MuxAlt4,                                           /* Pin is configured as SDHC0_D0 */
     kPORT_UnlockRegister                                     /* Pin Control Register fields [15:0] are not locked */
   };
   PORT_SetPinConfig(PORTE, 1, &porte1_pin2_config);   /* PORTE1 (pin 2) is configured as SDHC0_D0 */
   const port_pin_config_t porte2_pin3_config = {
     kPORT_PullUp,                                            /* Internal pull-up resistor is enabled */
     kPORT_FastSlewRate,                                      /* Fast slew rate is configured */
     kPORT_PassiveFilterDisable,                              /* Passive filter is disabled */
     kPORT_OpenDrainDisable,                                  /* Open drain is disabled */
     kPORT_HighDriveStrength,                                 /* High drive strength is configured */
     kPORT_MuxAlt4,                                           /* Pin is configured as SDHC0_DCLK */
     kPORT_UnlockRegister                                     /* Pin Control Register fields [15:0] are not locked */
   };
   PORT_SetPinConfig(PORTE, 2, &porte2_pin3_config);   /* PORTE2 (pin 3) is configured as SDHC0_DCLK */
   const port_pin_config_t porte3_pin4_config = {
     kPORT_PullUp,                                            /* Internal pull-up resistor is enabled */
     kPORT_FastSlewRate,                                      /* Fast slew rate is configured */
     kPORT_PassiveFilterDisable,                              /* Passive filter is disabled */
     kPORT_OpenDrainDisable,                                  /* Open drain is disabled */
     kPORT_HighDriveStrength,                                 /* High drive strength is configured */
     kPORT_MuxAlt4,                                           /* Pin is configured as SDHC0_CMD */
     kPORT_UnlockRegister                                     /* Pin Control Register fields [15:0] are not locked */
   };
   PORT_SetPinConfig(PORTE, 3, &porte3_pin4_config);   /* PORTE3 (pin 4) is configured as SDHC0_CMD */
   const port_pin_config_t porte4_pin5_config = {
     kPORT_PullUp,                                            /* Internal pull-up resistor is enabled */
     kPORT_FastSlewRate,                                      /* Fast slew rate is configured */
     kPORT_PassiveFilterDisable,                              /* Passive filter is disabled */
     kPORT_OpenDrainDisable,                                  /* Open drain is disabled */
     kPORT_HighDriveStrength,                                 /* High drive strength is configured */
     kPORT_MuxAlt4,                                           /* Pin is configured as SDHC0_D3 */
     kPORT_UnlockRegister                                     /* Pin Control Register fields [15:0] are not locked */
   };
   PORT_SetPinConfig(PORTE, 4, &porte4_pin5_config);   /* PORTE4 (pin 5) is configured as SDHC0_D3 */
   const port_pin_config_t porte5_pin6_config = {
     kPORT_PullUp,                                            /* Internal pull-up resistor is enabled */
     kPORT_FastSlewRate,                                      /* Fast slew rate is configured */
     kPORT_PassiveFilterDisable,                              /* Passive filter is disabled */
     kPORT_OpenDrainDisable,                                  /* Open drain is disabled */
     kPORT_HighDriveStrength,                                 /* High drive strength is configured */
     kPORT_MuxAlt4,                                           /* Pin is configured as SDHC0_D2 */
     kPORT_UnlockRegister                                     /* Pin Control Register fields [15:0] are not locked */
   };
   PORT_SetPinConfig(PORTE, 5, &porte5_pin6_config);   /* PORTE5 (pin 6) is configured as SDHC0_D2 */
   const port_pin_config_t porte6_pin7_config = {
     kPORT_PullDown,                                          /* Internal pull-down resistor is enabled */
     kPORT_FastSlewRate,                                      /* Fast slew rate is configured */
     kPORT_PassiveFilterDisable,                              /* Passive filter is disabled */
     kPORT_OpenDrainDisable,                                  /* Open drain is disabled */
     kPORT_LowDriveStrength,                                  /* Low drive strength is configured */
     kPORT_MuxAsGpio,                                         /* Pin is configured as PTE6 */
     kPORT_UnlockRegister                                     /* Pin Control Register fields [15:0] are not locked */
   };
   PORT_SetPinConfig(PORTE, 6, &porte6_pin7_config);   /* PORTE6 (pin 7) is configured as PTE6 */

   CLOCK_EnableClock(kCLOCK_Uart0);
   NVIC_SetPriority(UART0_RX_TX_IRQn, UART0_INT_PRIO);
}
