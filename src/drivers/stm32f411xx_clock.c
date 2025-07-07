


#include "drivers/stm32f411xx_clock.h"
#include <stdio.h>


/**
 * @brief  Configure and set all the necessary information for an External clock to be used
 * @param  pToClockHandler: Pointer to the Clock HSE handler structure
 * @retval None
 */
void ConfigureHSEClock(HSE_Clock_Handler_t *pToClockHandler) {

// first set the HSEBYP clock bit to inform the MC that the clock will be from an external source
pToClockHandler->pClock->RCC_CR |= RCC_CR_HSEBYP;

// disabling the clock from the external source and falling back to the internal HSI
pToClockHandler->pClock->RCC_CR &= ~RCC_CR_HSEON;

// Divide the clock from HSE to reach 1MHz for the internal RTC to function properly
pToClockHandler->pClock->RCC_CFGR |= (pToClockHandler->ClockConfig.ClockFreq << 16);

// Set the HSE as the main system clock
pToClockHandler->pClock->RCC_CFGR |= (ENABLE << 0);

// enables the clock from HSE source
pToClockHandler->pClock->RCC_CR |= RCC_CR_HSEON;

// wait for the clock to be ready
while (!pToClockHandler->pClock->RCC_CR & RCC_CR_HSERDY) {}

}