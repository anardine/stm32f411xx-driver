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

    SPI_PerClockControl(pToSPIHandle->pSPIx, ENABLE);
    
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

}

/**
 * @brief  Enable the SPI peripheral.
 * @param  pSPIx: Pointer to the SPI peripheral base address.
 * @retval None
 */
void SPI_Enable(SPI_Handle_t *pToSPIHandle) {

    // For convinience, this enables SSOE for STM32 being always the master. This needs to be changed if the STM32 will act as a Slave on any project.
    pToSPIHandle->pSPIx->SPI_CR2 |= (ENABLE << 2);


    pToSPIHandle->pSPIx->SPI_CR1 |= (ENABLE << 6); // enable SPI
}

/**
 * @brief  Disables the SPI peripheral
 * @param  pSPIx: Pointer to the SPI peripheral base address.
 * @retval None
 */
void SPI_Disable(SPI_Handle_t *pToSPIHandle) {

    while (((pToSPIHandle->pSPIx->SPI_SR & (1 << 7)) >> 7) == 1) // this can lock the application if the BSY flag is never cleared for some reason
    {
        printf("SPI is busy in communication or Tx buffer is not empty. Termination will happen when BSY is cleared");
    }

    pToSPIHandle->pSPIx->SPI_CR1 & ~(ENABLE << 6); // disable SPI

}

/**
 * @brief  Resets all registers of the specified SPI peripheral.
 * @param  pSPIx: Pointer to the SPI peripheral base address.
 * @retval None
 */
void SPI_DeInit(SPIx_MapR_t *pSPIx) {
    if (pSPIx == SPI1) {
        SPI1_REG_RESET();
    } else if (pSPIx == SPI2) {
        SPI2_REG_RESET();
    } else if (pSPIx == SPI3) {
        SPI3_REG_RESET();
    } else if (pSPIx == SPI4) {
        SPI4_REG_RESET();
    } else if (pSPIx == SPI5) {
        SPI5_REG_RESET();
    } else {
        printf("No SPI port recognized for reset. Please review the pointer to the correct SPI port");
    }

}

/**
 * @brief  Sends data over SPI.
 * @param  pSPIx: Pointer to the SPI peripheral base address.
 * @param  pToTrData: Pointer to the transmit buffer.
 * @param  Length: Number of bytes to send.
 * @retval None
 */
void SPI_SendData(SPIx_MapR_t *pSPIx, uint8_t *pToTrData, uint32_t Length) {
    while (Length > 0) {
        // Wait until TXE (Transmit buffer empty) flag is set
        while (!(pSPIx->SPI_SR & (1 << 1)));

        // Check DFF (Data Frame Format) bit to determine 8 or 16 bit data frame

        // A write to the data register will write into the Tx buffer and a read from the data register will return the value held in the Rx buffer.
        if (pSPIx->SPI_CR1 & (1 << 11)) {
            // 16-bit DFF
            pSPIx->SPI_DR = *((uint16_t *)pToTrData);
            pToTrData += 2;
            Length -= 2;
        } else {
            // 8-bit DFF
            pSPIx->SPI_DR = *pToTrData;
            pToTrData++;
            Length--;
        }
    }
}

/**
 * @brief  Receives data over SPI.
 * @param  pSPIx: Pointer to the SPI peripheral base address.
 * @param  pToRrBuffer: Pointer to the receive buffer.
 * @param  Length: Number of bytes to receive.
 * @retval None
 */
void SPI_ReceiveData(SPIx_MapR_t *pSPIx, uint8_t *pToRrBuffer, uint32_t Length) { // review code logic implementation
    while (Length > 0) {
        // Wait until RXNE (Receive buffer not empty) flag is set
        while (!(pSPIx->SPI_SR & (1 << 0)));

        // Check DFF (Data Frame Format) bit to determine 8 or 16 bit data frame
        if (pSPIx->SPI_CR1 & (1 << 11)) {
            // 16-bit DFF
            *((uint16_t *)pToRrBuffer) = (uint16_t)pSPIx->SPI_DR;
            pToRrBuffer += 2;
            Length -= 2;
        } else {
            // 8-bit DFF
            *pToRrBuffer = (uint8_t)pSPIx->SPI_DR;
            pToRrBuffer++;
            Length--;
        }
    }
}

/**
 * @brief  Configures and enables SPI interrupt for the specified SPI handle.
 * @param  pToSPIHandle: Pointer to the SPI handle structure.
 * @param  IRQ_SPI_h: Pointer to the IRQ handler structure.
 * @retval None
 */
void SPI_IRQInit(SPI_Handle_t *pToSPIHandle, IRQn_Handler_t *IRQ_SPI_h) { 
}

