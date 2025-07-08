#include "drivers/stm32f411xx_clock.h"
#include <stdio.h>

#ifdef USE_EXTERNAL_CLOCK

    /**
     * @brief  Configure and set all the necessary information for an External clock to be used
     * @param  pToClockHandler: Pointer to the Clock HSE handler structure
     * @retval None
     */
    void ConfigureHSEClock(HSE_Clock_Handler_t *pToClockHandler) {
        // Optional: Set HSEBYP if using external clock signal (not crystal)
        // pToClockHandler->pClock->RCC_CR |= RCC_CR_HSEBYP;

        // Enable HSE
        pToClockHandler->pClock->RCC_CR |= RCC_CR_HSEON;

        // Wait for HSE to be ready
        while (!(pToClockHandler->pClock->RCC_CR & RCC_CR_HSERDY)) {}

        pToClockHandler->pClock->RCC_CFGR |= (pToClockHandler->ClockConfig.ClockFreq << 16);

        // Select HSE as system clock source
        pToClockHandler->pClock->RCC_CFGR |= (1 << 0);

        // Optionally, wait for system clock switch to HSE to complete
        while (((pToClockHandler->pClock->RCC_CFGR >> 2) & 0x3) != 0x1) {}
    }

#endif