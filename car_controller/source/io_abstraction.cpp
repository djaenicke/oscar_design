/*
 * io_abstraction.cpp
 *
 *  Created on: Feb 9, 2019
 *      Author: djaenicke
 */

#include "io_abstraction.h"
#include "assert.h"

#define DEFAULT_GPIO_PIN_CFG \
{ \
   kPORT_PullDisable, \
   kPORT_FastSlewRate, \
   kPORT_PassiveFilterDisable, \
   kPORT_OpenDrainDisable, \
   kPORT_LowDriveStrength, \
   kPORT_MuxAsGpio, \
   kPORT_UnlockRegister \
} \

#define PULLUP_GPIO_PIN_CFG \
{ \
   kPORT_PullUp, \
   kPORT_FastSlewRate, \
   kPORT_PassiveFilterDisable, \
   kPORT_OpenDrainDisable, \
   kPORT_LowDriveStrength, \
   kPORT_MuxAsGpio, \
   kPORT_UnlockRegister \
} \


#define DEFAULT_ALT2_PIN_CFG \
{ \
   kPORT_PullDisable, \
   kPORT_FastSlewRate, \
   kPORT_PassiveFilterDisable, \
   kPORT_OpenDrainDisable, \
   kPORT_LowDriveStrength, \
   kPORT_MuxAlt2, \
   kPORT_UnlockRegister \
} \

#define DEFAULT_ALT3_PIN_CFG \
{ \
   kPORT_PullDisable, \
   kPORT_FastSlewRate, \
   kPORT_PassiveFilterDisable, \
   kPORT_OpenDrainDisable, \
   kPORT_LowDriveStrength, \
   kPORT_MuxAlt3, \
   kPORT_UnlockRegister \
} \

#define DEFAULT_ALT4_PIN_CFG \
{ \
   kPORT_PullDisable, \
   kPORT_FastSlewRate, \
   kPORT_PassiveFilterDisable, \
   kPORT_OpenDrainDisable, \
   kPORT_LowDriveStrength, \
   kPORT_MuxAlt4, \
   kPORT_UnlockRegister \
} \

#define DEFAULT_ANALOG_PIN_CFG \
{ \
   kPORT_PullDisable, \
   kPORT_FastSlewRate, \
   kPORT_PassiveFilterDisable, \
   kPORT_OpenDrainDisable, \
   kPORT_LowDriveStrength, \
   kPORT_PinDisabledOrAnalog, \
   kPORT_UnlockRegister \
} \

#define SDHC_PIN_CFG_1 \
{ \
   kPORT_PullUp, \
   kPORT_FastSlewRate, \
   kPORT_PassiveFilterDisable, \
   kPORT_OpenDrainDisable, \
   kPORT_HighDriveStrength, \
   kPORT_MuxAlt4, \
   kPORT_UnlockRegister \
} \

#define SDHC_PIN_CFG_2 \
{ \
   kPORT_PullDown, \
   kPORT_FastSlewRate, \
   kPORT_PassiveFilterDisable, \
   kPORT_OpenDrainDisable, \
   kPORT_LowDriveStrength, \
   kPORT_MuxAsGpio, \
   kPORT_UnlockRegister \
} \

const Pin_Cfg_T Pin_Cfgs[NUM_IO] =
{
   {PORTB, GPIOB, 21, DEFAULT_GPIO_PIN_CFG,   kGPIO_DigitalOutput, HIGH}, /* BLUE_LED - Active Low */
   {PORTA, GPIOA,  2, DEFAULT_ALT3_PIN_CFG,   kGPIO_DigitalOutput, LOW},  /* MOTOR_ENA - PWM */
   {PORTC, GPIOC,  2, DEFAULT_ALT4_PIN_CFG,   kGPIO_DigitalOutput, LOW},  /* MOTOR_ENB - PWM */
   {PORTC, GPIOC,  3, DEFAULT_GPIO_PIN_CFG,   kGPIO_DigitalOutput, LOW},  /* MOTOR_IN1 */
   {PORTD, GPIOD,  1, DEFAULT_GPIO_PIN_CFG,   kGPIO_DigitalOutput, LOW},  /* MOTOR_IN2 */
   {PORTC, GPIOC,  4, DEFAULT_GPIO_PIN_CFG,   kGPIO_DigitalOutput, LOW},  /* MOTOR_IN3 */
   {PORTD, GPIOD,  2, DEFAULT_GPIO_PIN_CFG,   kGPIO_DigitalOutput, LOW},  /* MOTOR_IN4 */
   {PORTA, GPIOA,  1, DEFAULT_GPIO_PIN_CFG,   kGPIO_DigitalOutput, LOW},  /* SERVO */
   {PORTC, GPIOC, 14, DEFAULT_ALT3_PIN_CFG,   kGPIO_DigitalInput,  NA},   /* UART4_RX */
   {PORTC, GPIOC, 15, DEFAULT_ALT3_PIN_CFG,   kGPIO_DigitalOutput, NA},   /* UART4_TX */
   {PORTB, GPIOB,  2, DEFAULT_ANALOG_PIN_CFG, kGPIO_DigitalOutput, NA},   /* VBATT */
   {PORTB, GPIOB, 18, DEFAULT_GPIO_PIN_CFG,   kGPIO_DigitalInput,  NA},   /* R_SPEED_SENSOR */
   {PORTC, GPIOC,  8, DEFAULT_GPIO_PIN_CFG,   kGPIO_DigitalInput,  NA},   /* L_SPEED_SENSOR */
   {PORTB, GPIOB, 11, DEFAULT_GPIO_PIN_CFG,   kGPIO_DigitalOutput, LOW},  /* Trigger Pin for USS */
   {PORTD, GPIOD,  3, DEFAULT_GPIO_PIN_CFG,   kGPIO_DigitalInput,  NA},   /* Echo Pin for USS */
   {PORTC, GPIOC, 10, DEFAULT_ALT2_PIN_CFG,   kGPIO_DigitalInput,  NA},   /* MPU6050 SCL */
   {PORTC, GPIOC, 11, DEFAULT_ALT2_PIN_CFG,   kGPIO_DigitalInput,  NA},   /* MPU6050 SDA */
   {PORTE, GPIOE,  0, SDHC_PIN_CFG_1,         kGPIO_DigitalOutput, NA},   /* SDHC0_D1 */
   {PORTE, GPIOE,  1, SDHC_PIN_CFG_1,         kGPIO_DigitalOutput, NA},   /* SDHC0_D0 */
   {PORTE, GPIOE,  2, SDHC_PIN_CFG_1,         kGPIO_DigitalOutput, NA},   /* SDHC0_DCLK */
   {PORTE, GPIOE,  3, SDHC_PIN_CFG_1,         kGPIO_DigitalOutput, NA},   /* SDHC0_CMD */
   {PORTE, GPIOE,  4, SDHC_PIN_CFG_1,         kGPIO_DigitalOutput, NA},   /* SDHC0_D3 */
   {PORTE, GPIOE,  5, SDHC_PIN_CFG_1,         kGPIO_DigitalOutput, NA},   /* SDHC0_D2 */
   {PORTE, GPIOE,  6, SDHC_PIN_CFG_2,         kGPIO_DigitalOutput, NA},   /* SDHC0_GPIO */
   {PORTB, GPIOB, 19, PULLUP_GPIO_PIN_CFG,    kGPIO_DigitalInput,  NA},   /* R_SPEED_SENSOR_HE (Hall-effect) */
   {PORTC, GPIOC,  1, PULLUP_GPIO_PIN_CFG,    kGPIO_DigitalInput,  NA},   /* L_SPEED_SENSOR_HE (Hall-effect) */
   {PORTB, GPIOB, 16, DEFAULT_ALT3_PIN_CFG,   kGPIO_DigitalInput,  NA},   /* UART0_RX */
   {PORTB, GPIOB, 17, DEFAULT_ALT3_PIN_CFG,   kGPIO_DigitalOutput, NA}    /* UART0_TX */
};

void Set_GPIO(IO_Map_T gpio, GPIO_State_T state)
{
    if (gpio < NUM_IO)
    {
        if ((kPORT_MuxAsGpio == Pin_Cfgs[gpio].pin_cfg.mux) &&
            (kGPIO_DigitalOutput == Pin_Cfgs[gpio].dir))
            GPIO_PinWrite(Pin_Cfgs[gpio].gbase, Pin_Cfgs[gpio].pin, (uint8_t) state);
        else
            assert(false);
    }
    else
        assert(false);
}

uint32_t Read_GPIO(IO_Map_T gpio)
{
    uint32_t ret_val = 0;

    if (gpio < NUM_IO)
    {
        if ((kPORT_MuxAsGpio == Pin_Cfgs[gpio].pin_cfg.mux) &&
            (kGPIO_DigitalInput == Pin_Cfgs[gpio].dir))
            ret_val = GPIO_PinRead(Pin_Cfgs[gpio].gbase, Pin_Cfgs[gpio].pin);
        else
            assert(false);
    }
    else
        assert(false);

    return (ret_val);
}

