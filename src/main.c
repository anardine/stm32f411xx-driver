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
 #include <stdio.h>
 #include "drivers/stm32f411xx_gpio.h"
 #include "drivers/stm32f411xx_intr.h"
 
 #if !defined(__SOFT_FP__) && defined(__ARM_FP)
   #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
 #endif
 
   GPIO_Handle_t ledHandler;

 int main(void)
 {

    // working on the LED exercise with the HAL implementation
    // LD2 on PA5 for NUCLEO-F411RE

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
  
  IRQn_Handler_t IRQ_GPIO_handler;

  GPIO_IRQInit(&buttonHandler, &IRQ_GPIO_handler);


  while (1) {

  }
  
 }

void EXTI_callback(void) {
  printf("button pressed! LED should toggle");
  GPIO_WriteToOutputPin(&ledHandler.pGPIOx,ledHandler.GPIO_PinConfig.GPIO_PinNumber, 1);
}

 void EXTI15_10_IRQHandler(void) {
    EXTI->EXTI_PR |= (1 << 13);
    EXTI_callback();
 }