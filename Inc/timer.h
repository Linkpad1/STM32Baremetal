/*
 * timer.h
 *
 *  Created on: Mar 3, 2024
 *      Author: Pee
 */

#ifndef TIMER_H_
#define TIMER_H_

void configureTimer2(void);
void configureTimer3(void);
void stop_timer(TIM_TypeDef *TIMx);
uint16_t read_TIM(TIM_TypeDef *TIMx);
void delay( uint32_t ms );

uint32_t DWT_Delay_Init(void); // HW Internal delay timer using system clock
void DWT_Delay_us(volatile uint32_t au32_microseconds);
void DWT_Delay_ms(volatile uint32_t au32_milliseconds);


#endif /* TIMER_H_ */
