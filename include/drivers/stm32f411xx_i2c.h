 /*
 * stm32f411xx_gpio.c
 *
 *  Created on: Jul 6, 2025
 *      Author: anardinelli
 */
 
 #ifndef INC_STM32F411XX_I2C_H_
 #define INC_STM32F411XX_I2C_H_


#include "drivers/stm32f411xx.h"
#include "drivers/stm32f411xx_clock.h"
#include <stdint.h>


  /**
     * @struct I2C_PinConfig_t
     * @brief Represents a structure with all the details needed for successfully cofiguring the I2C protocol
     * @param I2C_ClockSpeed The speed wanted. In standard mode (SM), this value normally is set to I2C_SLC_SPEED_NORMAL (which is 100kHz). For Fast Mode (FS), this is normally set to I2C_SLC_SPEED_FAST (400kHz)
     * @param I2C_DeviceAddress The device address if your configuring a slave
     * @param I2C_AckControl This is to enalbe/disabling acking. The dafault value is ENABLE. Only set to DISABLE if needed
     * @param I2C_DutyCycleForFastMode Only need to be set if using Fast Mode. Can be set I2C_DUTY_CYCLE_2 or I2C_DUTY_CYCLE_16_9
     * @param I2C_Mode You can either set I2C_SLC_MODE_STANDARD or I2C_SLC_MODE_FAST. Defaulted to I2C_SLC_MODE_STANDARD
    */
typedef struct 
{
    volatile uint8_t I2C_ClockSpeed;
    volatile uint8_t I2C_DeviceAddress;
    volatile uint8_t I2C_AckControl;
    volatile uint8_t I2C_DutyCycleForFastMode;
    volatile uint8_t I2C_Mode;

}I2C_PinConfig_t;


typedef struct stm32f411xx_i2c
{
    I2Cx_MapR_t *pI2Cx;
    I2C_PinConfig_t I2C_PinConfig;

}I2C_Handle_t;


void I2C_PerClockControl(I2Cx_MapR_t *pI2Cx, uint8_t ENorDI); //This takes a I2C address and an enable or disable flag to further enable the clock
 
#ifdef USE_EXTERNAL_CLOCK

    void I2C_Init(I2C_Handle_t *pToI2CHandle, HSE_Clock_Handler_t *pToClockHandler); // initialize all the characteristics of the I2C port when an external clock source is enabled

#else

    void I2C_Init(I2C_Handle_t *pToI2CHandle); // initialize all the characteristics of the I2C port with the default internal clock config

#endif

 void I2C_DeInit(I2Cx_MapR_t *pI2Cx); // resets all data from a specific I2C port



#endif