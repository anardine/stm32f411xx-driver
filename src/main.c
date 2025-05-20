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
    ledHandler.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_PD;

    GPIO_PerClockControl(ledHandler.pGPIOx, ENABLE);

    GPIO_Init(&ledHandler);

  // User button located at PC13. Should only tunr the LED on when pressed.
  GPIO_Handle_t buttonHandler;

  buttonHandler.pGPIOx = GPIOC;
  buttonHandler.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
  buttonHandler.GPIO_PinConfig.GPIO_PinNumber = 13;
  buttonHandler.GPIO_PinConfig.GPIO_isInterrupt = 1;
  
  GPIO_PerClockControl(buttonHandler.pGPIOx, ENABLE);
  
  GPIO_Init(&buttonHandler);


  while (1) {
    if (GPIO_ReadFromInputPin(buttonHandler.pGPIOx,13))
    {
      GPIO_WriteToOutputPin(ledHandler.pGPIOx, ledHandler.GPIO_PinConfig.GPIO_PinNumber, DISABLE);   
    } else {
      GPIO_WriteToOutputPin(ledHandler.pGPIOx, ledHandler.GPIO_PinConfig.GPIO_PinNumber, ENABLE);   
    }
  }
  return 0;
 }