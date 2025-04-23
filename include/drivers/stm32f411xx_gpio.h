/*
 * stm32f411xx_gpio.h
 *
 *  Created on: Apr 21, 2025
 *      Author: anardinelli
 */

 #ifndef INC_STM32F411XX_GPIO_H_
 #define INC_STM32F411XX_GPIO_H_
 
 #include "stm32f411xx.h"
 
 typedef struct {
     uint8_t GPIO_PinNumber;
     uint8_t GPIO_PinMode;
     uint8_t GPIO_PinSpeed;
     uint8_t GPIO_PinPuPdControl;
     uint8_t GPIO_PinOPType;
     uint8_t GPIO_PinAltFunMode;
 }GPIO_PinConfig_t;
 
 
 typedef struct {
     GPIOx_MapR_t *pGPIOx; //holds the base addr of GPIO port to witch the pin belongs
     GPIO_PinConfig_t GPIO_PinConfig; //this holds gpio pin config settings
 
 }GPIO_Handle_t;
 
 
 void GPIO_PerClockControl(GPIOx_MapR_t *pGPIOx, uint8_t ENorDI); //This takes a GPIO address and an enable or disable flag to further enable the clock
 
 void GPIO_Init(GPIO_Handle_t *pToGPIOHandle); // initialize all the characteristics of the GPIO port
 
 void GPIO_DeInit(GPIOx_MapR_t *pGPIOx); // resets all data from a specific GPIO port
 
 void GPIO_ReadFromInputPin(GPIOx_MapR_t *pGPIOx, uint8_t pinNumber);
 
 void GPIO_ReadFromInputPort(void);
 void GPIO_WriteToOutputPin(void);
 void GPIO_WriteToOutputPort(void);
 void GPIO_ToggleOutputPin(void);
 
 //Interrupts handling
 void GPIO_IRQConfig(void);
 void GPIO_IRQHandling(void);
 
 
 
 
 
 #endif /* INC_STM32F411XX_GPIO_H_ */