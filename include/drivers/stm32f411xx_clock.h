/*
 * stm32f411xx.h
 *
 *  Created on: Jul 6, 2025
 *      Author: anardinelli
 */

 #ifndef INC_STM32F411XX_CLOCK_H_
 #define INC_STM32F411XX_CLOCK_H_
 
  #include "drivers/stm32f411xx.h"
 #include <stdint.h>

 typedef struct
 {
    uint8_t ClockFreq;
 }HSE_Clock_Config_t;

 typedef struct
 {
    RCC_Map_t *pClock;
    HSE_Clock_Config_t ClockConfig;
 }HSE_Clock_Handler_t;
 
 

void ConfigureHSEClock(HSE_Clock_Handler_t *pToClockHandler);


 #endif