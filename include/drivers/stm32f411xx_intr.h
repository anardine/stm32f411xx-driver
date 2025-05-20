

#include "drivers/stm32f411xx.h"


typedef struct {
    IRQn_MapR_t IRQn;
    uint16_t posArray[6];
} IRQn_Handler_t;



void NVIC_SelectPos(IRQn_Handler_t IRQ_handler); // select the correct position for the registry configuration for clearing set
void NVIC_EnableIRQ(IRQn_Handler_t IRQ_handler); // enables the interrupt on the given line
void NVIC_DisableIRQ(IRQn_Handler_t IRQ_handler); // disables the interrupt on the given line
IRQn_Handler_t NVIC_InitIRQ(IRQn_MapR_t IRQ_Map); 