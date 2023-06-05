#include "stm32l4xx.h"   
#include "stdlib.h"
#include "stdint.h"
#include "math.h"
#include "PID.h"
volatile int encoderCount = 0;
volatile int encoderCountB = 0;
int currentStepPosition;
int angle2Step;
volatile uint8_t checkBStatus = 0;
volatile int checkBPid = 0;
volatile uint16_t adc_value;
int stepPosition = 0;
static volatile uint32_t micro = 0;
int test = 0;
double Kp1 = 0.1035;
double Kd1 = 0.008;
double Kp2 = 0.5;
double Kd2 = 0.005;
static int rotCtr = 0;
static double frequency =0;
uint8_t rotDir = 0;
uint16_t ARRset = 0 ;
int encoderValue_array[2] = {0,0};
int stepPosition_array[2] = {0,0};
int deltaT = 5;
// Configuration setup pin 
// IO PIN													PHYSICS PIN
// POD GPIOA_PIN_0							-> 		A0
// ENCODER_A GPIOA_PIN_1				-> 		A1
// ENCODER_B GPIOA_PIN_3				-> 		A2
// DIR STEPMOTOR GPIOA_PIN_4		->		A3
// STEP STEPMOTOR GPIOA_PIN_5		->		A4
// EN STEPMOTOR GPIOA_PIN_6			->		A5
//UART1 TX GPIOA_PIN_9					->		D1
//UART1 RX GPIOA_PIN_10				->		D0
// STATUS LED GPIOB_PIN_3				->		D13
//BUTTON CHANGE_STATUS GPIOB_PIN_4				->	D12
//BUTTON PID_CALIBERATION	GPIOB_PIN_5			-> 	D11


void clockinit(void);
void ADC_init(void);
void SysTick_Handler(void);
void TIM2_init(void);
void TIM7_IRQHandler(void);
void TIM7_init(void);
void TIM6_init(void);
void TIM6_DAC_IRQHandler(void);
void delayuS(unsigned int us);


//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
void clockinit(void){
	// system clocks init
	RCC->CR |= RCC_CR_MSION;
	RCC->CFGR &= ~RCC_CFGR_SW;
	while((RCC->CR & RCC_CR_MSIRDY) == 0); //wait until msi ready
	RCC->CR &= ~RCC_CR_MSIRANGE;
	RCC->CR |= RCC_CR_MSIRANGE_10; //48MHz
	RCC->CR |= RCC_CR_MSIRGSEL; //set to 1 -> setting MSIRANGE in CR reg
	while((RCC->CR & RCC_CR_MSIRDY) == 0); //wait until msi ready	
	RCC->AHB2ENR |= 3;			//	enable GPIOA,GPIOB
	RCC->APB2ENR |= (1<<0); // enable SYSCFG
	RCC->APB1ENR1 |= (1<<0); //Timer 2
	RCC->APB1ENR1 |= (1<<5); //Timer 7
	RCC->APB1ENR1 |= (1<<4);//Timer 6
	// enable UART clock
	
}
void TIM2_init(void){ //for pin A5 as PWM AF - Tim2 channel 1 
	//	prescale
	RCC->CFGR |= (5<<11); //1/4 => timefrequency = 32MHz*2/4 = 16Mz
	TIM2->PSC = 8;     // 1/8 => timefrequency = 16MHz/8 = 2MHz -> 1tick = 0.5us
	//default TIM2->CR1 set DIR = 0 for upcounting
	//set pulse width
	TIM2->ARR = 1999; // 1 pulse = (1999+1)*0.5us = 1ms
	TIM2->CCR1 = 0;
	TIM2->CNT =0;
	TIM2->CCMR1 |= (6<<4);  
	TIM2->CCMR1 |= (1<<3);
	TIM2->CR1 |= (1<<7);
	TIM2->CCER |= (1<<0); 
}
void TIM7_init(void){ // Timer use to calculate the step of stepper motor
	//Set APB1 prescaler and Timer prescaler ---- 1 step = 8 pulse
	TIM7->PSC = 32; 
	TIM7->ARR =  1999;   // 1 interupt = (1999+1)*2 = 4ms
	TIM7->CNT = 0;
	TIM7->DIER |= (1<<0); 
	__enable_irq(); 
	NVIC_EnableIRQ(TIM7_IRQn); 
}
void TIM6_init(void){ // working as timmer clock
	TIM6->PSC = 2; // f = 16MHz*2/(4*2) = 4MHz --> 4 tick = 1us
	TIM6->ARR = 40; // 10us for 1 interupt
	TIM6->CNT = 0;
	TIM6->DIER |= (1<<0); // enable interupt
	__enable_irq(); // Enable PRIMASK 
	NVIC_EnableIRQ(TIM6_DAC_IRQn); // enable NVIC
}
//-----------------------------------------------------------------------
//----------------------For calib PID (ADC)------------------------------
void ADC1_IRQHandler (void);
void ADC_init(void){
	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
	RCC->CCIPR |= (3<<28);        
	ADC1_COMMON->CCR &= ~(3u<<16); 
	ADC1_COMMON->CCR |= (1<<18);  
	ADC1->CR &= ~ADC_CR_DEEPPWD; 
	ADC1->CR |= ADC_CR_ADVREGEN;
	// Set as discontinuous mode - default ADC_CFGR CONT =0
	ADC1->CFGR &= ~(1u<<5); //Right ALIGN
	ADC1->CFGR &= ~(3u<<3);	//12bit 
	ADC1->SMPR1 = (4<<3);				 
	ADC1->SQR1 &= ~(1u<<0); 
	ADC1->SQR1 |= (5<<6);		
	ADC1->IER |= ADC_IER_EOCIE; 		 
	while(ADC1->CR & ADC_CR_ADCAL); 

}

void ADC1_IRQHandler (void){
	// read ADC value and clear EOC flag
	if (abs((int)ADC1->DR - (int)adc_value)>1)
	adc_value = ADC1->DR;
	ADC1->CR &= ~ADC_CR_ADSTART;  
}

//-------------------------------------------------------------------------------------------
//---------------------For Encoder Reading --------------------------------------------------
// interupt for both encoder channel for both rising and falling detection
void EXTI1_IRQHandler(void);
void EXTI3_IRQHandler(void);
void refEncoderHandler(void);
uint8_t Mask = 15; //0b00001111 -> get 4 value on the right
uint8_t refEncode = 0 ;
void EXTI1_IRQHandler(void){
	EXTI->PR1|=(1<<1);
	uint8_t EA = (GPIOA->IDR)&(1<<1);
	uint8_t EB = ((GPIOA->IDR)&(1<<3))>>3;
	uint8_t encode = EA|EB;
	refEncode = Mask & ((refEncode<<2) | (encode));
	if(refEncode == 0b1101 || refEncode == 0b0100  || refEncode == 0b0010 || refEncode == 0b1011) {
    encoderCount++; //CW
  }
	if(refEncode  == 0b1110 || refEncode == 0b0111 || refEncode == 0b0001 || refEncode == 0b1000) {
    encoderCount--;  //CCW
  }
	if( abs(encoderCount)>800 ) encoderCount = -encoderCount + 2*encoderCount%800; // if encoderCount = 802 => encoderCount = -802 + 2*2 = -798;
																																								//  if encoderCount = -802 => encoderCount = 802 + 2*-2 = 798;
}
void EXTI3_IRQHandler(void){
	EXTI->PR1|=(1<<3);
	uint8_t EA = (GPIOA->IDR)&(1<<1);
	uint8_t EB = ((GPIOA->IDR)&(1<<3))>>3;
	uint8_t encode = EA|EB;
	refEncode = Mask & ((refEncode<<2) | (encode));
	if(refEncode == 0b1101 || refEncode == 0b0100  || refEncode == 0b0010 || refEncode == 0b1011) {
    encoderCount++; //CW
  }
	if(refEncode  == 0b1110 || refEncode == 0b0111 || refEncode == 0b0001 || refEncode == 0b1000) {
    encoderCount--;  //CCW
  }
	if( abs(encoderCount)>800 ) encoderCount = -encoderCount + 2*encoderCount%800; 
}

//-----------------------------------------------------------------
//---------------------Button Function-----------------------------
//this button is used to reset the original value of the system
void EXTI9_5_IRQHandler(void);
void EXTI9_5_IRQHandler(void){
	EXTI->PR1|=(1<<5);
	EXTI->PR1|=(1<<9);
	encoderCount = 800;
	encoderValue_array[0] = 0;
	encoderValue_array[1] = 0;
	stepPosition = 0;
	stepPosition_array[0] = 0;
	stepPosition_array[1] = 0;
	
}

//--------------------------------------------------------------------------
//---------------------For controlling Step motor---------------------------
void Rotate_Angle(uint16_t angle, uint32_t ARR_value, uint8_t DIR);
void Rotate(uint32_t ARR_value, uint8_t DIR);
void returnHome(void);

void Rotate(uint32_t ARR_value, uint8_t DIR){
		if (ARR_value > 1){
		TIM2->ARR  = ARR_value-1;
		TIM2->CCR1 = (uint32_t)(TIM2->ARR*3/4);
		TIM7->ARR = ARR_value -1;
		if (DIR == 1) GPIOA->ODR |= (1<<4);
		else GPIOA->ODR &= ~(1<<4);
		TIM2->CR1 |= (1<<0);
		TIM7->CR1 |= (1<<0);
		}
		else{
			TIM2->CCR1 = 0;
			TIM7->CR1 &= ~(1<<0);
			TIM2->CR1 |= (1<<0);
			
		}
}
void Rotate_Angle(uint16_t angle, uint32_t ARR_value, uint8_t DIR){
	angle2Step = (uint16_t)(angle*400/360);
	currentStepPosition = stepPosition;
	if (ARR_value != 0){
		TIM2->ARR  = ARR_value-1;
		TIM2->CCR1 = (uint32_t)(TIM2->ARR*3/4);
		TIM7->ARR = ARR_value -1;
	}
	else{
		TIM2->ARR  = 0;
		TIM2->CCR1 = 0;
		TIM7->ARR = 0;
	}
	if (DIR == 1) GPIOA->ODR |= (1<<4);
	else GPIOA->ODR &= ~(1<<4);
	TIM2->CR1 |= (1<<0);
	TIM7->CR1 |= (1<<0);	
	while ((abs(stepPosition - currentStepPosition)) <= angle2Step);
	Rotate( 0,DIR); // stop
	// CLOSE the timer
	TIM2->CR1 &= ~(1U<<0);
	TIM7->CR1 &= ~(1U<<0);
	TIM6->CR1 &= ~(1U<<0); 	
}

void returnHome(void){
	while(stepPosition != 0){
	if (stepPosition > 0) Rotate(7000,1);
	else  Rotate(7000,0);
	}
	Rotate(0,0);
}

void TIM7_IRQHandler(void) {
	TIM7->SR = 0;
	if ((GPIOA->ODR & (1<<4)) == 0)
		stepPosition++;
	else stepPosition--;
}

void TIM6_DAC_IRQHandler(void) {
	TIM6->SR = 0;
	micro +=10;
}

void delayuS(unsigned int us){
	//Enable counter
	TIM6 ->CR1 |=(1<<0);
	micro = 0;
	while (micro < us); // delay time
	micro = 0;
	//disable counter
	TIM6->CR1 &=~(1u<<0); 
	TIM6->CNT = 0;
}
//--------------------------------------------------------------------------
//---------------------- Update PID value ---------------------------------
double ABSf(double fnum); // get absolute value of float
void updateFrequency(void);

double ABSf(double fnum){
	if (fnum<0) fnum = -fnum;
	return fnum;
}

void updateFrequency(void){
	double frequency1 = PD(Kp1,Kd1, 0, frequency, encoderValue_array, deltaT); 
	//double frequency2 = PD(Kp2,Kd2, 0, frequency, stepPosition_array, deltaT); 
	frequency = frequency1;
		// ARR max = 65536 = 65536 -> min frequency = 1
	// ARR min = 2000 -> max frequency = ARRmax/ARRmin = 32.768
	if (ABSf(frequency )<= 1){rotCtr = 0;}
	else 
	{
		if (frequency > 33) frequency = 33;
		else if (frequency < -33){ frequency = -33;}
		
		if (frequency  > 0) {
			rotDir = 0;
			rotCtr = (int)(65536/frequency);
		}
		else {
			rotDir = 1;
			rotCtr = (int)(-65536/frequency);
		}
	}
}
void SysTick_Handler(void){
// read ADC value
	ADC1->CR |= ADC_CR_ADSTART; 
	updateValue2(encoderCount,encoderValue_array);
	deltaT = 5;
	updateFrequency();
	ADC1->CR |= ADC_CR_ADSTART;
}

//================================MAIN FUNCTION========================================
int main(void)
{
	SysTick_Config(125000); // sample freq = 256Hz
	clockinit();

	//-------------GPIO setup--------------//
	//A0: Analog Mode		(11)
	//A1: Input Mode		(00) 
	//A3: Input Mode		(00)
	//A4: Output Mode		(01)
	//A5: AF Mode-TIM2	(10) 
	//A6: Output Mode		(01)
	//B4: Input Mode		(00)
	//B5: Input Mode		(00)
	GPIOA->MODER = 0xAB7FD933; 
	
	GPIOA->AFR[0] |= (1<<20); 
	GPIOB->MODER = 0xFFFFF2BF; 

	
	//----------------Function init------------------//
	ADC_init();
	TIM7_init();
	TIM2_init();
	TIM6_init();
	__enable_irq();
	NVIC_EnableIRQ(ADC1_IRQn);
	// start calibration and enable
	ADC1->CR |= ADC_CR_ADCAL;       
	while(ADC1->CR & ADC_CR_ADCAL); 
	// enable ADC
	ADC1->CR |= ADC_CR_ADEN;  
	while(!(ADC1->ISR & ADC_ISR_ADRDY));  
	ADC1->CR |= ADC_CR_ADSTART; 
	
	
	//-------------enable EXTERNAL Interupts---------//
	//EXTI1 PINA -> ENCODER A
	//EXTI3 PINA -> ENCODER B
	//EXTI4 PINB-> BUTTON CHANGE STATUS
	//EXTI5 PINB-> BUTTON CHANGE PID
	SYSCFG->EXTICR[0]=0; 
	SYSCFG->EXTICR[1]=0x10; 
	EXTI->IMR1|=(1<<1)|(1<<3)|(1<<5);// disable mask
	EXTI->RTSR1 |= (1<<1)|(1<<3)|(1<<5);//enable raising trigger	
	EXTI->FTSR1 |= (1<<1)|(1<<3)|(1<<5);//enable falling trigger	
	__enable_irq();
	NVIC_ClearPendingIRQ(EXTI1_IRQn);
	NVIC_ClearPendingIRQ(EXTI3_IRQn);
	NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_EnableIRQ(EXTI3_IRQn);
	NVIC_EnableIRQ(EXTI9_5_IRQn);

	TIM2->CR1 |= (1<<0);
	encoderCount = 800;
	GPIOA->ODR |= (1<<11);
	while(1)
	{
			while (abs(encoderCount)<170){
				frequency = 0;
				Kp1 = 0.002;
				Kd1 = 0;
			while (abs(encoderCount)<150){
					Rotate(abs(rotCtr),rotDir);
					while (abs(encoderCount)<2){
						Kp1 = 0.1035;
						Kd1 = 0.008;
						while (abs(encoderCount)<60){ Rotate(abs(rotCtr),rotDir);}
					}	
				}
			}
			Rotate(0,0);
	
	}
}