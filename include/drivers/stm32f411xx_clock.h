/*
 * stm32f411xx.h
 *
 *  Created on: Jul 6, 2025
 *      Author: anardinelli
 */

 /* This define statement is required when using an external clock configuration. 
 After this definition, the HSE_Clock_Handler_t must be instantiated, and the 
 ConfigureHSEClock(HSE_Clock_Handler_t *pToClockHandler) function should be called.
 If you do not define it, the code will use all clock settings as the default for the stm32f411 board. */
 #define USE_EXTERNAL_CLOCK

 #ifndef INC_STM32F411XX_CLOCK_H_
 #define INC_STM32F411XX_CLOCK_H_

#ifndef USE_EXTERNAL_CLOCK
#define USE_INTERNAL_CLOCK
#endif
 
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