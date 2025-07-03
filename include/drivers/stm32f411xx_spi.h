 /*
 * stm32f411xx_gpio.c
 *
 *  Created on: Jun 18, 2025
 *      Author: anardinelli
 */
 
 #ifndef INC_STM32F411XX_SPI_H_
 #define INC_STM32F411XX_SPI_H_


#include "drivers/stm32f411xx.h"
#include <stdint.h>

typedef struct
{
    volatile uint8_t SPI_DeviceMode;   // SPI device mode: master or slave
    volatile uint8_t SPI_BusConfig;    // SPI bus configuration: full-duplex, half-duplex, simplex
    volatile uint8_t SPI_SclckSpeed;   // SPI serial clock speed (baud rate prescaler)
    volatile uint8_t SPI_DFF;          // SPI data frame format: 8-bit or 16-bit
    volatile uint8_t SPI_CPOL;         // SPI clock polarity: idle high or low
    volatile uint8_t SPI_CPHA;         // SPI clock phase: first or second clock transition
    volatile uint8_t SPI_SSM;          // SPI software slave management: enabled or disabled

} SPI_PinConfig_t; // Structure for configuring SPI peripheral settings


typedef struct 
{
    SPIx_MapR_t *pSPIx; //holds the base addr of SPI port to witch the pin belongs
    SPI_PinConfig_t SPI_PinConfig; //this holds SPI pin config settings

} SPI_Handle_t;


 void SPI_PerClockControl(SPIx_MapR_t *pSPIx, uint8_t ENorDI); //This takes a SPI address and an enable or disable flag to further enable the clock
 
 void SPI_Init(SPI_Handle_t *pToSPIHandle); // initialize all the characteristics of the SPI port
 
 void SPI_DeInit(SPIx_MapR_t *pSPIx); // resets all data from a specific SPI port

 void SPI_Enable(SPI_Handle_t *pToSPIHandle); // Enables the specified SPI peripheral.

 void SPI_Disable(SPI_Handle_t *pToSPIHandle); // Disables the specified SPI peripheral.

void SPI_SendData(SPIx_MapR_t *pSPIx, uint8_t *pToTrBuffer, uint32_t Length); 

void SPI_ReceiveData(SPIx_MapR_t *pSPIx, uint8_t *pToRrBuffer, uint32_t Length);

 void SPI_IRQInit(SPI_Handle_t *pToSPIHandle,  IRQn_Handler_t *IRQ_SPI_h);

 #endif /* INC_STM32F411XX_SPI_H_ */