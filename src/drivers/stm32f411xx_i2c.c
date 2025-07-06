/*
 * stm32f411xx_gpio.c
 *
 *  Created on: Jul 6, 2025
 *      Author: anardinelli
 */


  #include "drivers/stm32f411xx_i2c.h"
 #include <stdio.h>

 /**
 * @brief  Enables or disables the peripheral clock for the given I2C port.
 * @param  pI2Cx: Pointer to the I2C peripheral base address.
 * @param  ENorDI: Enable or Disable flag (1 = Enable, 0 = Disable).
 * @retval None
 */
void I2C_PerClockControl(I2Cx_MapR_t *pI2Cx, uint8_t ENorDI) {

    if (ENorDI == ENABLE) {
        if (pI2Cx == I2C1) {
            I2C1_CLK_EN();
        } else if (pI2Cx == I2C2) {
            I2C2_CLK_EN();
        } else if (pI2Cx == I2C3) {
            I2C3_CLK_EN();
        } else {
            printf("No I2C port recognized when turning clock ON. Please review the pointer to the correct I2C port");
        }
    } else {
        if (pI2Cx == I2C1) {
            I2C1_CLK_DIS();
        } else if (pI2Cx == I2C2) {
            I2C2_CLK_DIS();
        } else if (pI2Cx == I2C3) {
            I2C3_CLK_DIS();
        } else {
            printf("No I2C port recognized when turning clock OFF. Please review the pointer to the correct I2C port");
        }
    }
}

/**
 * @brief  Initializes the I2C peripheral with the specified settings.
 * @param  pToI2CHandle: Pointer to the I2C handle structure containing configuration info.
 * @retval None
 */
void I2C_Init(I2C_Handle_t *pToI2CHandle) {

    I2C_PerClockControl(pToI2CHandle->pI2Cx, ENABLE);
    
    //reset the CR1 and CR2 register to default
    pToI2CHandle->pI2Cx->I2C_CR1 = 0;
    pToI2CHandle->pI2Cx->I2C_CR2 = 0;

    // further initialization logic here


}

/**
 * @brief  Resets all registers of the specified I2C peripheral.
 * @param  pI2Cx: Pointer to the I2C peripheral base address.
 * @retval None
 */
void I2C_DeInit(I2Cx_MapR_t *pI2Cx) {
    if (pI2Cx == I2C1) {
        I2C1_REG_RESET();
    } else if (pI2Cx == I2C2) {
        I2C2_REG_RESET();
    } else if (pI2Cx == I2C3) {
        I2C3_REG_RESET();
    } else {
        printf("No I2C port recognized for reset. Please review the pointer to the correct I2C port");
    }
}
