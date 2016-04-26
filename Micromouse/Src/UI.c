#include "stm32f4xx_hal.h"
#include "gpio_r.h"
#include "UI.h"

void UI_Init()
{
	UI_InitBeep();
	UI_InitLeds();
}


void UI_InitBeep()
{
	__TIM1_CLK_ENABLE();
	__TIM2_CLK_ENABLE();
	__GPIOA_CLK_ENABLE();
	__DSB(); // Data Synchronization Barrier
	
	gpio_pin_cfg(GPIOA, PA5, gpio_mode_AF1_PP_LS);
	
	
	//--- TIM1 master mode
	/*TIM1->CR1 |= TIM_CR1_DIR | TIM_CR1_OPM; // downcounter, one pulse mode
	//TIM1->CR2 = TIM_CR2_MMS_1; // CEN is used as TRGO
	TIM1->ARR = 5500; // duration*/
	// wlacz UEV interrupt to set TIM2->CNT = 0 albo 0xffff ==> celar SR_UIF
	//TIM1->DIER = TIM_DIER_UIE;
	//NVIC_EnableIRQ(TIM1_UP_TIM10_IRQHandler);
	
	//--- TIM2 CH1 config (freq)
	// select OCxM
	//TIM2->CNT = 5000; // (32bit)
	TIM2->ARR = 14e2; // 1k4 -> 60kHz (32bit) (max)
	TIM2->PSC = 150; // current freq = 60kHz/150 = 400Hz
	//TIM2->CR1 = TIM_CR1_ARPE; // ARR shadow register
	TIM2->CCR2 = TIM2->ARR/2; // 50% duty
	TIM2->CCMR1 = TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_0; // PWM Mode 1
	TIM2->CCER = TIM_CCER_CC2E; // CC2E-output enable, CC1P - polarity
	//TIM2->BDTR |= TIM_BDTR_MOE; // MOE-main output enable
	
	//TS=000 -> Master is TIM1
	//TS=001 -> Master is TIM8
	//TIM2->SMCR |= TIM_SMCR_SMS_2|TIM_SMCR_SMS_0; //TS=000 SMS=101 - gated mode
	
	// enable timers
	TIM2->CR1 |= TIM_CR1_CEN;
}

void UI_InitLeds()
{
	// config timers in One Pulse mode
	UI_LedOff();
}

void UI_Beep(int time, int freq)
{
	//freq to TIM2
	TIM2->PSC = 60e3 / freq + 1;
	
	//time to TIM1
	TIM1->CNT = time;
	TIM1->CR1 |= TIM_CR1_CEN;
}

void UI_LedOff()
{

}


void UI_Send(uint8_t* m)
{
	HAL_UART_Transmit(&UI_UART_Handle, m, strlen((char*)m), 0xFF);
}