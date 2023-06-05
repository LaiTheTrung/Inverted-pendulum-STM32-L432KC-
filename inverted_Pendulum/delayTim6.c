#include "delayTim6.h"                // Device header
void TIM6_init(void){
	//Timer 6 enable
	RCC->APB1ENR1 |= (1<<4);
	//Set APB1 prescaler and Timer prescaler
	TIM6->PSC = 2; // f = 16MHz*2/(4*2) = 4MHz --> 4 tick = 1us
	//Configure the timer 6
	TIM6->ARR = 40; // 10us for 1 interupt
	TIM6->CNT = 0;
	TIM6->DIER |= (1<<0); // enable interupt
	//Enable PRIMASK and NVIC
	__enable_irq(); // Enable PRIMASK 
	NVIC_EnableIRQ(TIM6_DAC_IRQn); // enable NVIC
}
void TIM6_DAC_IRQHandler(void) {
	TIM6->SR = 0;
	micro += 10;
}

void delayuS(unsigned int us){
	//Enable counter
	TIM6 ->CR1 |=(1<<0);
	micro = 0;
	while (micro < us); // delay time
	//disable counter
	TIM6->CR1 &=~(1u<<0); 
	TIM6->CNT = 0;
}
