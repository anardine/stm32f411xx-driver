/*
 * stm32f411xx.h
 *
 *  Created on: Jul 6, 2025
 *      Author: anardinelli
 */

 /** 
  * @attention This define statement is required when using an external clock configuration. After this definition, the  HSE_Clock_Handler_t must be instantiated, and the ConfigureHSEClock(HSE_Clock_Handler_t *pToClockHandler) function should be called. If you do not define it, the code will use all clock settings as the default for the stm32f411 board. 
 */
 #define USE_EXTERNAL_CLOCK

 #ifndef INC_STM32F411XX_CLOCK_H_
 #define INC_STM32F411XX_CLOCK_H_

#ifndef USE_EXTERNAL_CLOCK
    #define USE_INTERNAL_CLOCK
    #warning "The clock being used is the INTERNAL clock source (16MHz) HSI. If you want to use an enxternal source please define USE_EXTERNAL_CLOCK at "include/drivers/stm32f411xx_clock.h"
#endif
 
  #include "drivers/stm32f411xx.h"
 #include <stdint.h>

    #ifdef USE_EXTERNAL_CLOCK
    #warning "This framework is set to be used with an external clock source (HSE). Please use the HSE_Clock_Handler_t to define the correct details and call ConfigureHSEClock(HSE_Clock_Handler_t *pToClockHandler) to define the external clock details before implemention.

    /** 
     * @attention Define the frequency of HSE_Clock_Config_t.ClockFreq using an unsigned int in the MHz Range. 
     * @example An 8MHz external clock, the value of HSE_Clock_Config_t.ClockFreq should be only 8 (eight).
     * @note As an example, for an 8MHz external clock, the value of HSE_Clock_Config_t.ClockFreq should be only 8 (eight).
     */
    typedef struct
    {
        uint8_t ClockFreq;
    }HSE_Clock_Config_t;

    /**
     * @struct HSE_Clock_Handler_t
     * @brief Represents a handler for configuring an external clock source
     * @param pClock The pointer to the RCC configuration registers
     * @param ClockConfig The detials of the clock configuration
     * @attention If you plan to enable I2C, please notice that the HSE frequency must be at least 2 MHz to achieve Sm mode I²C frequencies. It must be at least 4MHz to achieve Fm mode I²C frequencies. It must be a multiple of 10MHz to reach the
    400 kHz maximum I²C Fm mode clock.
    */
    typedef struct
    {
        RCC_Map_t *pClock;
        HSE_Clock_Config_t ClockConfig;
    }HSE_Clock_Handler_t;
    
    
    void ConfigureHSEClock(HSE_Clock_Handler_t *pToClockHandler);

    #endif

#endif