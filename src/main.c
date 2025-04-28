/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Alessandro Nardinelli
 * @brief          : Main program body
 ******************************************************************************
 *
 ******************************************************************************
 */

 #include <stdint.h>
 #include "drivers/stm32f411xx_gpio.h"
 
 #if !defined(__SOFT_FP__) && defined(__ARM_FP)
   #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
 #endif
 
 int main(void)
 {

    // working on the LED exercise with the HAL implementation
    // LD2 on PA5 for NUCLEO-F411RE

    GPIO_Handle_t ledHandler;

    ledHandler.pGPIOx = GPIOA;

    ledHandler.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
    ledHandler.GPIO_PinConfig.GPIO_PinNumber = 5;
    ledHandler.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PP;

    GPIO_PerClockControl(ledHandler.pGPIOx, ENABLE);

    while (1)
    {
      for(int32_t i = 0; i < 3000000; i++);
      GPIO_WriteToOutputPin(ledHandler.pGPIOx, ledHandler.GPIO_PinConfig.GPIO_PinNumber, ENABLE);
      for(int32_t i = 0; i < 3000000; i++);
      GPIO_WriteToOutputPin(ledHandler.pGPIOx, ledHandler.GPIO_PinConfig.GPIO_PinNumber, DISABLE);    
    }
    
     return 0;
 }