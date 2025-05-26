

#include "drivers/stm32f411xx_intr.h"
#include <stdio.h>


 /****************************************************************
  * @name				-NVIC_SelectPos
  *
  * @brief 				- This function selects which position of the NVIC registers will be addressed to be set
  *
  * @param[in]			- the vector table IRQ handler
  *
  * @return				- none. It populates the array passed with the data needed to instantiate the NVIC controller
  *
  * @note				- none
  *
  * */
void NVIC_SelectPos(IRQn_Handler_t *IRQ_handler) {

    if (IRQ_handler->IRQn <= 31) {
        for(int i = 0U; i < 4; i++) {
            IRQ_handler->posArray[i] = 0; //sets all positions of the array with 0 
        }
    } else if (IRQ_handler->IRQn > 31 && IRQ_handler->IRQn <=63)
    {
        for(int i = 0U; i < 4; i++) {
        IRQ_handler->posArray[i] = 1; //sets all positions of the array with 1 
        }
    } else if (IRQ_handler->IRQn > 63 && IRQ_handler->IRQn <= 96)
    {
        for(int i = 0U; i < 4; i++) {
        IRQ_handler->posArray[i] = 2; //sets all positions of the array with 2
        }
    } else {
        printf("implementation on stm32f411xx only have 93 interrupts. Setting further NVIC positions than NVIC_2 is not requeried.");
    }

    IRQ_handler->posArray[4] = IRQ_handler->IRQn/4; //set the correct priority register to be selected

    IRQ_handler->posArray[5] = 0; // always set the last postion to zero given there's only one location to STIR
}


 /****************************************************************
  * @name				-NVIC_EnableIRQ
  *
  * @brief 				- This enables the NVIC on the set IRQ number given
  *
  * @param[in]			- the vector table IRQ handler
  *
  * @return				- none
  *
  * @note				- none
  *
  * */
void NVIC_EnableIRQ(IRQn_Handler_t *IRQ_handler) {

    uint16_t bitToSet = IRQ_handler->IRQn % 8;

    NVIC->NVIC_ISER[IRQ_handler->posArray[0]] |= (1 << bitToSet);
    
}

 /****************************************************************
  * @name				-NVIC_DisableIRQ
  *
  * @brief 				- This disables the NVIC on the set IRQ number given
  *
  * @param[in]			- the vector table IRQ handler
  *
  * @return				- none
  *
  * @note				- none
  *
  * */
void NVIC_DisableIRQ(IRQn_Handler_t *IRQ_handler) {
    
    uint16_t bitToSet = IRQ_handler->IRQn % 8;
    
    NVIC->NVIC_ICER[IRQ_handler->posArray[1]] |= (1 << bitToSet);

    
}

 /****************************************************************
  * @name				-NVIC_PriorityIRQ
  *
  * @brief 				- This sets the NVIC priority
  *
  * @param[in]			- the vector table IRQ handler
  *
  * @return				- none
  *
  * @note				- none
  *
  * */
void NVIC_PriorityIRQ(IRQn_Handler_t *IRQn_Handler, uint32_t priority) {

    uint16_t sectionToSet = IRQn_Handler->IRQn % 4;

    NVIC->NVIC_IPR[IRQn_Handler->posArray[4]] |= (priority << sectionToSet);

}