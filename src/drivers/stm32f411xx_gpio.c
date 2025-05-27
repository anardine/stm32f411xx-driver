/*
 * stm32f411xx_gpio.c
 *
 *  Created on: Apr 21, 2025
 *      Author: anardinelli
 */

 #include "drivers/stm32f411xx_gpio.h"
 #include <stdio.h>
 #include <stdlib.h>

 
 
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
  * @note				- none
  *
  * */
 void GPIO_PerClockControl(GPIOx_MapR_t *pGPIOx, uint8_t ENorDI) {
 
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
  * @note				- none
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
 

void GPIO_IRQInit(GPIO_Handle_t *pToGPIOHandler,  IRQn_Handler_t *IRQ_GPIO_h) {

    volatile uint8_t pinToSet = pToGPIOHandler->GPIO_PinConfig.GPIO_PinNumber;

        if(pToGPIOHandler->GPIO_PinConfig.GPIO_isInterrupt == 1) { // this marks the necessity of setting the interrupt registers given that the user wants to set an interrupt
               
        // enable clock on SYSCFG if not enabled yet
        RCC->RCC_APB2ENR |= (1 << 14);    
        
        // defines the port of the passed pin to set the correct EXT line. This is done using the SYSCFG_EXTCR register
        uint8_t EXTIcrNumber = pinToSet/4;
        uint8_t bitToSet = pinToSet%4*4;
        
        // identify which is the correct port for setting the interrupt
        uint8_t portToSet;

        if(pToGPIOHandler->pGPIOx == GPIOA) { // this can be further optimized to become a macro in stm32f411xx.h file
            portToSet = 0x0;
        } else if (pToGPIOHandler->pGPIOx == GPIOB) {
            portToSet = 0x1;
        } else if (pToGPIOHandler->pGPIOx == GPIOC) {
            portToSet = 0x2;
        } else if (pToGPIOHandler->pGPIOx == GPIOD) {
            portToSet = 0x3;
        } else if (pToGPIOHandler->pGPIOx == GPIOE) {
            portToSet = 0x4;
        } else if (pToGPIOHandler->pGPIOx == GPIOH) {
            portToSet = 0x7;
        } else {
            printf("cannot open port: Unrecognized referece. Please set GPIO to be A, B, C, D, E or H");
            portToSet = 0x0;
        }

        SYSCFG->SYSCFG_EXTCRx[3] |= (portToSet << bitToSet); // define val according to the port received 
        
        // set the interrupt mask register
        EXTI->EXTI_IMR |= (1 << pinToSet);

        //set the trigger selection register (defaulted to always Rising Edge detection)
        EXTI->EXTI_FTSR |= (1 << pinToSet);

        // set the correct IRQ Handler to init the pin
        if (pinToSet == 0) {
            IRQ_GPIO_h->IRQn = EXTI0_IRQn;
        } else if (pinToSet == 1) {
            IRQ_GPIO_h->IRQn = EXTI1_IRQn;
        } else if (pinToSet == 2) {
            IRQ_GPIO_h->IRQn = EXTI2_IRQn;
        } else if (pinToSet == 3) {
            IRQ_GPIO_h->IRQn = EXTI3_IRQn;
        } else if (pinToSet == 4) {
            IRQ_GPIO_h->IRQn = EXTI4_IRQn;
        } else if (pinToSet >= 5 && pinToSet <= 9) {
            IRQ_GPIO_h->IRQn = EXTI9_5_IRQn;
        } else if (pinToSet >= 10 && pinToSet <= 15) {
            IRQ_GPIO_h->IRQn = EXTI15_10_IRQn;
        } else {
            printf("IRQ_Hander out of range. Please check if GPIO pin is less or equal to 15");
        }
    }

//     NVIC_SelectPos(IRQ_GPIO_h);
//     NVIC_PriorityIRQ(IRQ_GPIO_h, 20);
//     NVIC_EnableIRQ(IRQ_GPIO_h); // enables the NVIC
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
  * @note				- none
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
  * @note				- none
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
  * @note				- none
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
  * @note				- none
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
  * @note				- none
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
  * @note				- none
  *
  * */
 void GPIO_ToggleOutputPin(GPIOx_MapR_t *pGPIOx, uint8_t pinNumber){
    pGPIOx->ODR ^= (1 << pinNumber);

    //000101001001010010011  - bit
    //000000000000000000001  - mask (pretending on bit pos 0)
    //000101001001010010010  - RESULT: XOR will only return 1 when it's either 1 on the bit or the mask, and not both. You can use to write 1 or 0 everytime you call the function to that pin


 }
