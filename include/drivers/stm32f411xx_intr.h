

#include "drivers/stm32f411xx.h"

void NVIC_SelectPos(IRQn_Handler_t *IRQ_handler); // select the correct position for the registry configuration for clearing set
void NVIC_EnableIRQ(IRQn_Handler_t *IRQ_handler); // enables the interrupt on the given line
void NVIC_DisableIRQ(IRQn_Handler_t *IRQ_handler); // disables the interrupt on the given line