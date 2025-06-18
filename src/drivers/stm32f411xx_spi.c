/*
 * stm32f411xx_gpio.c
 *
 *  Created on: Jun 18, 2025
 *      Author: anardinelli
 */

 #include "drivers/stm32f411xx_spi.h"
 
/**
 * @brief  Enables or disables the peripheral clock for the given SPI port.
 * @param  pSPIx: Pointer to the SPI peripheral base address.
 * @param  ENorDI: Enable or Disable flag (1 = Enable, 0 = Disable).
 * @retval None
 */
void SPI_PerClockControl(SPIx_MapR_t *pSPIx, uint8_t ENorDI) {

    if (ENorDI == ENABLE) {
        if (pSPIx == SPI1) {
            SPI1_CLK_EN();
        } else if (pSPIx == SPI2) {
            SPI2_CLK_EN();
        } else if (pSPIx == SPI3) {
            SPI3_CLK_EN();
        } else if (pSPIx == SPI4) {
            SPI4_CLK_EN();
        } else if (pSPIx == SPI5) {
            SPI5_CLK_EN();
        } else {
            printf("No SPI port recognized when turning clock ON. Please review the pointer to the correct SPI port");
        }
    } else {
        if (pSPIx == SPI1) {
            SPI1_CLK_DIS();
        } else if (pSPIx == SPI2) {
            SPI2_CLK_DIS();
        } else if (pSPIx == SPI3) {
            SPI3_CLK_DIS();
        } else if (pSPIx == SPI4) {
            SPI4_CLK_DIS();
        } else if (pSPIx == SPI5) {
            SPI5_CLK_DIS();
        } else {
            printf("No SPI port recognized when turning clock OFF. Please review the pointer to the correct SPI port");
        }
    }
}

/**
 * @brief  Initializes the SPI peripheral with the specified settings.
 * @param  pToSPIHandle: Pointer to the SPI handle structure containing configuration info.
 * @retval None
 */
void SPI_Init(SPI_Handle_t *pToSPIHandle) {
    

    


}

/**
 * @brief  Resets all registers of the specified SPI peripheral.
 * @param  pSPIx: Pointer to the SPI peripheral base address.
 * @retval None
 */
void SPI_DeInit(SPIx_MapR_t *pSPIx) {
    // Function implementation goes here
}

/**
 * @brief  Sends data over SPI.
 * @param  pSPIx: Pointer to the SPI peripheral base address.
 * @param  pToTrBuffer: Pointer to the transmit buffer.
 * @param  Length: Number of bytes to send.
 * @retval None
 */
void SPI_SendData(SPIx_MapR_t *pSPIx, uint8_t *pToTrBuffer, uint32_t Length) {
    // Function implementation goes here
}

/**
 * @brief  Receives data over SPI.
 * @param  pSPIx: Pointer to the SPI peripheral base address.
 * @param  pToRrBuffer: Pointer to the receive buffer.
 * @param  Length: Number of bytes to receive.
 * @retval None
 */
void SPI_ReceiveData(SPIx_MapR_t *pSPIx, uint8_t *pToRrBuffer, uint32_t Length) {
    // Function implementation goes here
}

/**
 * @brief  Configures and enables SPI interrupt for the specified SPI handle.
 * @param  pToSPIHandle: Pointer to the SPI handle structure.
 * @param  IRQ_SPI_h: Pointer to the IRQ handler structure.
 * @retval None
 */
void SPI_IRQInit(SPI_Handle_t *pToSPIHandle, IRQn_Handler_t *IRQ_SPI_h) {
    // Function implementation goes here
}

