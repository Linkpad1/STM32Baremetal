/*
 * timer_interrupt.h
 *
 *  Created on: Mar 6, 2024
 *      Author: Pee
 */

#include "stm32f4xx.h"


#ifndef TIMER_INTERRUPT_H_
#define TIMER_INTERRUPT_H_

void start_timer(TIM_TypeDef *TIMx, uint16_t ms);
void enable_Timer_4 (void);
void ConfigureTimer5(void);

#endif /* TIMER_INTERRUPT_H_ */
