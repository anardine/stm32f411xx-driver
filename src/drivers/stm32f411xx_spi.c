/*
 * stm32f411xx_gpio.c
 *
 *  Created on: Jun 18, 2025
 *      Author: anardinelli
 */

 #include "drivers/stm32f411xx_spi.h"
 #include <stdio.h>
 
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
    
    //reset the CR1 register to default
     pToSPIHandle->pSPIx->SPI_CR1 = 0;

    // Set the SPI clock phase (CPHA) bit in CR1 register according to configuration
    pToSPIHandle->pSPIx->SPI_CR1 |= (pToSPIHandle->SPI_PinConfig.SPI_CPHA << 0);
    
    // Set the SPI clock polarity (CPOL) bit in CR1 register according to configuration
    pToSPIHandle->pSPIx->SPI_CR1 |= (pToSPIHandle->SPI_PinConfig.SPI_CPOL << 1);

    // Set the SPI device mode (master/slave) bit in CR1 register according to configuration
    pToSPIHandle->pSPIx->SPI_CR1 |= (pToSPIHandle->SPI_PinConfig.SPI_DeviceMode << 2);

    //configure the device to work on full duplex, half duplex or simplex RX only
    if ((pToSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_FULL_DUPLEX) || (pToSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_HALF_DUPLEX)) {
        pToSPIHandle->pSPIx->SPI_CR1 |= (pToSPIHandle->SPI_PinConfig.SPI_BusConfig << 15);
    } else if ((pToSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX)) {
        pToSPIHandle->pSPIx->SPI_CR1 &= ~(pToSPIHandle->SPI_PinConfig.SPI_BusConfig << 15); // set bidmode to 2-line unidirectional data mode
        pToSPIHandle->pSPIx->SPI_CR1 |= (pToSPIHandle->SPI_PinConfig.SPI_BusConfig << 10); // set Output to disabled (Receive-only mode)
    }
    
    // Set the SPI serial clock speed (baud rate prescaler) bits in CR1 register according to configuration
    pToSPIHandle->pSPIx->SPI_CR1 |= (pToSPIHandle->SPI_PinConfig.SPI_SclckSpeed << 3);

    //select if the dataframe should be of 8 or 16 bits
    pToSPIHandle->pSPIx->SPI_CR1 |= (pToSPIHandle->SPI_PinConfig.SPI_DFF << 11);

    pToSPIHandle->pSPIx->SPI_CR1 |= (ENABLE << 6); // enable SPI

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

