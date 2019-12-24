#ifndef _IO_ABSTRACTION_H_
#define _IO_ABSTRACTION_H_

#include <stdint.h>

#include "fsl_port.h"
#include "fsl_gpio.h"


typedef enum
{
    LOW=0,
    HIGH=1,
    NA
} GPIO_State_T;

typedef struct
{
    PORT_Type *pbase;
    GPIO_Type *gbase;
    uint32_t   pin;
    port_pin_config_t pin_cfg;
    gpio_pin_direction_t dir;
    GPIO_State_T init_state;
} Pin_Cfg_T;

typedef enum
{
    BLUE_LED=0,
    MOTOR_ENA,
    MOTOR_ENB,
    MOTOR_IN1,
    MOTOR_IN2,
    MOTOR_IN3,
    MOTOR_IN4,
    SERVO,
    UART4_RX,
    UART4_TX,
    VBATT,
    USS_TRIGGER,
    USS_ECHO,
    MPU6050_SCL,
    MPU6050_SDA,
    SDHC0_D1,
    SDHC0_D0,
    SDHC0_DCLK,
    SDHC0_CMD,
    SDHC0_D3,
    SDHC0_D2,
    SDHC0_GPIO,
    R_SPEED_SENSOR,
    L_SPEED_SENSOR,
    UART0_RX,
    UART0_TX,
    RMII0_RXD1,
    RMII0_RXD0,
    RMII0_CRS_DV,
    RMII0_TXEN,
    RMII0_TXD0,
    RMII0_TXD1,
    RMII0_RXER,
    RMII0_MDC,
    ENET0_1588_TMR0,
    ENET0_1588_TMR1,
    ENET0_1588_TMR2,
    PTA4,
    RMII0_MDIO,
    NUM_IO /* !!! Make sure this is last */
} IO_Map_T;

extern const Pin_Cfg_T Pin_Cfgs[NUM_IO];

extern void Set_GPIO(IO_Map_T gpio, GPIO_State_T state);
extern uint32_t Read_GPIO(IO_Map_T gpio);

#endif /* _IO_ABSTRACTION_H_ */
