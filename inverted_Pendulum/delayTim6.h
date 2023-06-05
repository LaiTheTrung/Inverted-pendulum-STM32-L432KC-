#include "stm32l4xx.h"  
void TIM6_init(void);
void TIM6_DAC_IRQHandler(void);
void delayuS(unsigned int ms);
static volatile uint32_t micro = 0;