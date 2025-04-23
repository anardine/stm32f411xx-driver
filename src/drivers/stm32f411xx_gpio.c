/*
 * stm32f411xx_gpio.c
 *
 *  Created on: Apr 21, 2025
 *      Author: anardinelli
 */

 #include "drivers/stm32f411xx_gpio.h"
 #include <stdio.h>
 
 
 /****************************************************************
  * @name				-GPIO_PerClockControl
  *
  * @brief 				- This function enables or disables peripheral clock for the given GPIO port
  *
  * @param[in]			- pointer to the base address of the gpio peripheral
  * @param[in]			- ENNABLE or DISABLE flag
  *
  * @return				- void
  *
  * @notes				- none
  *
  * */
 
 void GPIO_PerClockControl(GPIOx_MapR_t *pGPIOx, uint8_t ENorDI){
 
     if (ENorDI == ENABLE)
     {
         if (pGPIOx == GPIOA) {
             GPIOA_CLK_EN();
         } else if (pGPIOx == GPIOB) {
             GPIOB_CLK_EN();
         } else if (pGPIOx == GPIOC) {
             GPIOC_CLK_EN();
         } else if (pGPIOx == GPIOD) {
             GPIOD_CLK_EN();
         } else if (pGPIOx == GPIOE) {
             GPIOE_CLK_EN();
         } else if (pGPIOx == GPIOH) {
             GPIOH_CLK_EN();
         } else {
             printf("No GPIO port recognized when turning clock ON. Please review the pointer to the correct GPIO port");
         }
     } else {
         if (pGPIOx == GPIOA) {
             GPIOA_CLK_DIS();
         } else if (pGPIOx == GPIOB) {
             GPIOB_CLK_DIS();
         } else if (pGPIOx == GPIOC) {
             GPIOC_CLK_DIS();
         } else if (pGPIOx == GPIOD) {
             GPIOD_CLK_DIS();
         } else if (pGPIOx == GPIOE) {
             GPIOE_CLK_DIS();
         } else if (pGPIOx == GPIOH) {
             GPIOH_CLK_DIS();
         } else {
             printf("No GPIO port recognized when turning clock OFF. Please review the pointer to the correct GPIO port");
         }
     }
 }
 
 
 /****************************************************************
  * @name				-GPIO_Init
  *
  * @brief 				- This function Init a GPIO port by setting the properties defined at the handler
  *
  * @param[in]			- pointer to the handler of the gpio peripheral
  *
  * @return				- void
  *
  * @notes				- none
  *
  * */
 void GPIO_Init(GPIO_Handle_t *pToGPIOHandle) {
 
     // identify what is the pin number that the user wants to set
     uint8_t pinToSet = pToGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
     uint8_t ModerPinPosition = pinToSet *2; // MODER have 2 bit settings.
     uint8_t SpeedPinPosition = pinToSet *2; // SPEED have 2 bit settings.
     uint8_t PuPdPinPosition = pinToSet *2; // PullUp PullDown have 2 bit settings.
     uint8_t AltFuncPosition = 0;
 
     if(pinToSet <=7)
     {
         AltFuncPosition = pinToSet *4;
 
     } else {
         switch(pinToSet) {
         case 8: AltFuncPosition = 0;
         break;
         case 9: AltFuncPosition = 4;
         break;
         case 10: AltFuncPosition = 8;
         break;
         case 11: AltFuncPosition = 12;
         break;
         case 12: AltFuncPosition = 16;
         break;
         case 13: AltFuncPosition = 20;
         break;
         case 14: AltFuncPosition = 24;
         break;
         case 15: AltFuncPosition = 28;
         break;
         }
     }
 
 
     //set the MODER register to the value defined at the moder definition of the Handler
     uint8_t pinMode = pToGPIOHandle->GPIO_PinConfig.GPIO_PinMode; // can be set to 0 (input), 1 (output), 2 (Alt function) or 3 (Analog)
     pToGPIOHandle->pGPIOx->MODER |= ( pinMode << ModerPinPosition);
 
     //set the output type defined by the user
     uint8_t outputType = pToGPIOHandle->GPIO_PinConfig.GPIO_PinOPType; // can be 0 (push-pull) or 1 (open-drain)
     pToGPIOHandle->pGPIOx->OTYPER |= (outputType << pinToSet);
 
     uint8_t pinSpeed = pToGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed; // can be 0 (low), 1 (medium), 2 (fast) and 3 (high)
     pToGPIOHandle->pGPIOx->OSPEEDR |= (pinSpeed << SpeedPinPosition);
 
     uint8_t puPdMode = pToGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl; // can be 0 (no pupd), 1 (up), 2 (down)
     pToGPIOHandle->pGPIOx->OSPEEDR |= (puPdMode << PuPdPinPosition);
 
     uint8_t altMode = pToGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode; // varies accornding to the function setting the user wants, ranging from 0000 to 1111. Check the reference manual for information
 
     if(pinToSet <= 7)
     {
         pToGPIOHandle->pGPIOx->AFRL |= (altMode << AltFuncPosition);
     }
     else
     {
         pToGPIOHandle->pGPIOx->AFRH |= (altMode << AltFuncPosition);
     }
 
 }
 
 
 
 
 /****************************************************************
  * @name				-GPIO_DeInit
  *
  * @brief 				- This function resets all charactesitiscs form a GPIO port
  *
  * @param[in]			- pointer to the base address of the gpio peripheral
  *
  * @return				- void
  *
  * @notes				- none
  *
  * */
 void GPIO_DeInit(GPIOx_MapR_t *pGPIOx) {
 
     if (pGPIOx == GPIOA) {
                 RST_GPIOA();
             } else if (pGPIOx == GPIOB) {
                 RST_GPIOB();
             } else if (pGPIOx == GPIOC) {
                 RST_GPIOC();
             } else if (pGPIOx == GPIOD) {
                 RST_GPIOD();
             } else if (pGPIOx == GPIOE) {
                 RST_GPIOE();
             } else if (pGPIOx == GPIOH) {
                 RST_GPIOH();
             } else {
                 printf("No GPIO port recognized to reset. Please review the pointer to the correct GPIO port");
             }
 
 }