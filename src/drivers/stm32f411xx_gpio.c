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
 
     //set the MODER register to the value defined at the moder definition of the Handler
     uint8_t volatile pinMode = pToGPIOHandle->GPIO_PinConfig.GPIO_PinMode; // can be set to 0 (input), 1 (output), 2 (Alt function) or 3 (Analog)
     pToGPIOHandle->pGPIOx->MODER |= ( pinMode << ModerPinPosition);
 
     //set the output type defined by the user
     uint8_t volatile outputType = pToGPIOHandle->GPIO_PinConfig.GPIO_PinOPType; // can be 0 (push-pull) or 1 (open-drain)
     pToGPIOHandle->pGPIOx->OTYPER |= (outputType << pinToSet);
 
     uint8_t volatile pinSpeed = pToGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed; // can be 0 (low), 1 (medium), 2 (fast) and 3 (high)
     pToGPIOHandle->pGPIOx->OSPEEDR |= (pinSpeed << SpeedPinPosition);
 
     uint8_t volatile puPdMode = pToGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl; // can be 0 (no pupd), 1 (up), 2 (down)
     pToGPIOHandle->pGPIOx->OSPEEDR |= (puPdMode << PuPdPinPosition);

     if(pToGPIOHandle->GPIO_PinConfig.GPIO_PinMode == 2) { // this defends the necessity of setting the alternate function registers only if the mode is set to alternate function
       
        uint8_t AltFuncPosition = 0; // define the alternate function position map depeding on what was assigned to pin map
        uint8_t altMode = pToGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode; // varies accornding to the function setting the user wants, ranging from 0000 to 1111. Check the reference manual for information
       
        if(pinToSet <=7) {
            AltFuncPosition = pinToSet *4;
            pToGPIOHandle->pGPIOx->AFRL |= (altMode << AltFuncPosition);
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
            pToGPIOHandle->pGPIOx->AFRH |= (altMode << AltFuncPosition);
        }
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


  /****************************************************************
  * @name				-GPIO_ReadFromInputPin
  *
  * @brief 				- This function returns the read bit from the GPIO and pin number provided
  *
  * @param[in]			- pointer to the base address of the gpio peripheral
  * @param[in]			- pin number for reading
  * 
  * @return				- 0 or 1 when data is read
  *
  * @notes				- none
  *
  * */
 uint8_t GPIO_ReadFromInputPin(GPIOx_MapR_t *pGPIOx, uint8_t pinNumber) {
    
    int8_t volatile x = (pGPIOx->IDR >> pinNumber); 
    x &= (1 << 0); 
   return x;
 }


  /****************************************************************
  * @name				-GPIO_ReadFromInputPort
  *
  * @brief 				- This function returns the entire 32bit value from the read of any given GPIO port
  *
  * @param[in]			- pointer to the base address of the gpio peripheral
  * 
  * @return				- the entire 32 bit value from the IDR register for a given port
  *
  * @notes				- none
  *
  * */
 uint32_t GPIO_ReadFromInputPort(GPIOx_MapR_t *pGPIOx) {

    int32_t valueAtGPIOx = pGPIOx->IDR;
    return valueAtGPIOx;

 }



 /****************************************************************
  * @name				-GPIO_WriteToOutputPin
  *
  * @brief 				- This function sets the output bit of the port and data provided
  *
  * @param[in]			- pointer to the base address of the gpio peripheral
  * @param[in]			- the pin number to set
  * @param[in]			- the data to write to the output mode
  * 
  * @return				- none
  *
  * @notes				- none
  *
  * */
 void GPIO_WriteToOutputPin(GPIOx_MapR_t *pGPIOx, uint8_t pinNumber, uint8_t dataToWrite) {

    uint8_t volatile outputMode = (pGPIOx->MODER >> pinNumber*2); 
    outputMode &= (3 << 0); //check to see if there's anything stored in MODER defined as output

    if(outputMode == 1) { //only proceeds if MODER is set to output on pin selected
        (dataToWrite == DISABLE) ? (pGPIOx->ODR &= ~(1 << pinNumber)) : (pGPIOx->ODR |= (dataToWrite << pinNumber));
    } else {
        printf("Unable to set output register. Please set the MODER registry as output mode (01) before setting output bits");
    }

 }

/****************************************************************
  * @name				-GPIO_WriteToOutputPort
  *
  * @brief 				- This function sets the entire output 32-bit reg of the port
  *
  * @param[in]			- pointer to the base address of the gpio peripheral
  * @param[in]			- the data to write to the output mode
  * 
  * @return				- none
  *
  * @notes				- none
  *
  * */
 void GPIO_WriteToOutputPort(GPIOx_MapR_t *pGPIOx, uint32_t dataToWrite) {

    // reset the entire ODR given that we're going to re-write the whole 32 bit
    pGPIOx->ODR |= dataToWrite;

 }


/****************************************************************
  * @name				-GPIO_ToggleOutputPin
  *
  * @brief 				- This function toggles the output pin
  *
  * @param[in]			- pointer to the base address of the gpio peripheral
  * @param[in]			- the pin number to write to the output mode
  * 
  * @return				- none
  *
  * @notes				- none
  *
  * */
 void GPIO_ToggleOutputPin(GPIOx_MapR_t *pGPIOx, uint8_t pinNumber){
    pGPIOx->ODR ^= (1 << pinNumber);

    //000101001001010010011  - bit
    //000000000000000000001  - mask (pretending on bit pos 0)
    //000101001001010010010  - RESULT: XOR will only return 1 when it's either 1 on the bit or the mask, and not both. You can use to write 1 or 0 everytime you call the function to that pin


 }
