/*
 * io_abstraction.cpp
 *
 *  Created on: Feb 9, 2019
 *      Author: djaenicke
 */

#include "io_abstraction.h"
#include "assert.h"

const Pin_Cfg_T Pin_Cfgs[NUM_IO] =
{
    {PORTB, GPIOB, 21, kPORT_MuxAsGpio,           kGPIO_DigitalOutput, HIGH}, /* BLUE_LED - Active Low */
    {PORTA, GPIOA,  2, kPORT_MuxAlt3,             kGPIO_DigitalOutput, LOW},  /* MOTOR_ENA - PWM */
    {PORTC, GPIOC,  2, kPORT_MuxAlt4,             kGPIO_DigitalOutput, LOW},  /* MOTOR_ENB - PWM */
    {PORTC, GPIOC,  3, kPORT_MuxAsGpio,           kGPIO_DigitalOutput, LOW},  /* MOTOR_IN1 */
    {PORTD, GPIOD,  1, kPORT_MuxAsGpio,           kGPIO_DigitalOutput, LOW},  /* MOTOR_IN2 */
    {PORTC, GPIOC,  4, kPORT_MuxAsGpio,           kGPIO_DigitalOutput, LOW},  /* MOTOR_IN3 */
    {PORTD, GPIOD,  2, kPORT_MuxAsGpio,           kGPIO_DigitalOutput, LOW},  /* MOTOR_IN4 */
    {PORTA, GPIOA,  1, kPORT_MuxAsGpio,           kGPIO_DigitalOutput, LOW},  /* SERVO */
    {PORTC, GPIOC, 14, kPORT_MuxAlt3,             kGPIO_DigitalInput,  NA},   /* UART4_RX */
    {PORTC, GPIOC, 15, kPORT_MuxAlt3,             kGPIO_DigitalOutput, NA},   /* UART4_TX */
    {PORTB, GPIOB,  2, kPORT_PinDisabledOrAnalog, kGPIO_DigitalOutput, NA},   /* VBATT */
    {PORTB, GPIOB, 18, kPORT_MuxAsGpio,           kGPIO_DigitalInput,  NA},   /* R_SPEED_SENSOR */
    {PORTC, GPIOC,  8, kPORT_MuxAsGpio,           kGPIO_DigitalInput,  NA},   /* L_SPEED_SENSOR */
    {PORTC, GPIOC, 10, kPORT_MuxAsGpio,           kGPIO_DigitalOutput, LOW},  /* Trigger Pin for USS */
    {PORTD, GPIOD,  3, kPORT_MuxAsGpio,           kGPIO_DigitalInput,  NA},   /* Echo Pin for USS */
};

void Set_GPIO(IO_Map_T gpio, GPIO_State_T state)
{
    if (gpio < NUM_IO)
    {
        if ((kPORT_MuxAsGpio == Pin_Cfgs[gpio].mux) &&
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
        if ((kPORT_MuxAsGpio == Pin_Cfgs[gpio].mux) &&
            (kGPIO_DigitalInput == Pin_Cfgs[gpio].dir))
            ret_val = GPIO_PinRead(Pin_Cfgs[gpio].gbase, Pin_Cfgs[gpio].pin);
        else
            assert(false);
    }
    else
        assert(false);

    return (ret_val);
}

