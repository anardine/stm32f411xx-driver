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

  
    
  while (1) {

  }
  
 }

void EXTI_callback(void) {
  printf("button pressed! LED should toggle");
  GPIO_WriteToOutputPin(ledHandler.pGPIOx,ledHandler.GPIO_PinConfig.GPIO_PinNumber, ENABLE);
}

 void EXTI15_10_IRQHandler(void) {
  if ((EXTI->EXTI_PR & (1<<13)) != 0) {
    
  EXTI->EXTI_PR |= (1 << 13);
    EXTI_callback();
  }
 }