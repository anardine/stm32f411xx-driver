 /*
 * stm32f411xx_gpio.c
 *
 *  Created on: Jul 6, 2025
 *      Author: anardinelli
 */
 
 #ifndef INC_STM32F411XX_SPI_H_
 #define INC_STM32F411XX_SPI_H_


#include "drivers/stm32f411xx.h"
#include <stdint.h>

typedef struct 
{
    volatile uint8_t I2C_ClockSpeed;
    volatile uint8_t I2C_DeviceAddress;
    volatile uint8_t I2C_Acking;
    volatile uint8_t I2C_Mode;
    volatile uint8_t I2C_DutyCycleForFastMode;

}I2C_PinConfig_t;


typedef struct stm32f411xx_i2c
{
    I2Cx_MapR_t *pI2Cx;
    I2C_PinConfig_t I2C_PinConfig;

}I2C_Handle_t;






#endif