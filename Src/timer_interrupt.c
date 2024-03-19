/*
 * timer_interrupt.c
 *
 *  Created on: Mar 6, 2024
 *      Author: Pee
 */

#include "timer_interrupt.h"


#define int32_t         int
#define int16_t         short
#define int8_t          char
#define uint32_t        unsigned int
#define uint16_t        unsigned short
#define uint8_t         unsigned char


void start_timer(TIM_TypeDef *TIMx, uint16_t ms) {
  // Start by making sure the timer's 'counter' is off.
  TIMx->CR1 &= ~(TIM_CR1_CEN);
  // Next, reset the peripheral. (This is where a HAL can help)
  if (TIMx == TIM2) {
    RCC->APB1RSTR |=  (RCC_APB1RSTR_TIM2RST);
    RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM2RST);
  }
  #ifdef VVC_F0
  else if (TIMx == TIM14) {
    RCC->APB1RSTR |= (RCC_APB1RSTR_TIM14RST);
    RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM14RST);
  }
  else if (TIMx == TIM16) {
    RCC->APB2RSTR |=  (RCC_APB2RSTR_TIM16RST);
    RCC->APB2RSTR &= ~(RCC_APB2RSTR_TIM16RST);
  }
  else if (TIMx == TIM17) {
    RCC->APB2RSTR |=  (RCC_APB2RSTR_TIM17RST);
    RCC->APB2RSTR &= ~(RCC_APB2RSTR_TIM17RST);
  }
  #endif
  // Set the timer prescaler/autoreload timing registers.
  // (These are 16-bit timers, so this won't work with >65MHz.)
  TIMx->PSC   = 15999; //
  TIMx->ARR   = ms;
  // Send an update event to reset the timer and apply settings.
  TIMx->EGR  |= TIM_EGR_UG;
  // Enable the hardware interrupt.
  TIMx->DIER |= TIM_DIER_UIE;
  // Enable the timer.
  TIMx->CR1  |= TIM_CR1_CEN;
}



void ConfigureTimer5(void)
{
	  /* Enable the APB clock FOR TIM4  */
	  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM5EN);

  TIM5->PSC = 15999;

  /* (1 KHz / 1000) = 1Hz = 1s */
  /* So, this will generate the 1s delay */
  TIM5->ARR = 2999;

  /* Enable the Interrupt */
  TIM5->DIER |= TIM_DIER_UIE;

  /* Clear the Interrupt Status */
  TIM5->SR &= ~TIM_SR_UIF;    // Clear the interrupt

  /* Enable NVIC Interrupt for Timer 3 */
  NVIC_EnableIRQ(TIM5_IRQn);
  NVIC_SetPriority (TIM5_IRQn, 3);  // Set Priority
  /* Finally enable TIM3 module */
  TIM5->CR1 = TIM_CR1_CEN;
}



void enable_Timer_4 (void)
{
		// Enable the TIM4 clock.
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	// Enable the NVIC interrupt for TIM4.
	NVIC_SetPriority (TIM4_IRQn, 3);  // Set Priority
	NVIC_EnableIRQ(TIM4_IRQn);
}



