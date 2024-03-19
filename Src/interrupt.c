/*
 * Interrupt.c
 *
 *  Created on: Mar 4, 2024
 *      Author: Pee
 */

#include "interrupt.h"
#include "usart.h"


void enable_interrupt(IRQn_Type IRQn)
{
	NVIC->ISER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}

void disable_interrupt(IRQn_Type IRQn)
{
	NVIC->ICER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}



void interrupt_setup(void)
{
	//PG0 [EXTI 0] [keypad]
	//PG1 [EXTI 1] [keypad]

	// Set Bit 5 to enable GPIOG clock
	RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOGEN_Pos);

	// Make GPIOG Pin0 and Pin1 input
	GPIOG->MODER &= ~GPIO_MODER_MODER0;  // SET BIT 0 to 0
	GPIOG->MODER &= ~GPIO_MODER_MODER1;  // SET BIT 1 to 0

	//The SYSCFG peripheral is in the ‘APB2’ clock domain
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	//
	SYSCFG->EXTICR[0]  |= SYSCFG_EXTICR1_EXTI0_PG;
	SYSCFG->EXTICR[0]  |= SYSCFG_EXTICR1_EXTI1_PG;

	// Next we choose either rising edge trigger (RTSR) or falling edge trigger (FTSR)
	EXTI->RTSR |= EXTI_RTSR_TR0;   // Enable rising edge trigger on EXTI0
	EXTI->RTSR |= EXTI_RTSR_TR1;  // Enable rising edge trigger on EXTI1

	// We mask the used external interrupt numbers.
	EXTI->IMR |= 0x00001;    // Mask EXTI0
	EXTI->IMR |= 0x00002;    // Mask EXTI1

	// Set Prioirity for each interrupt request
	// STM32F107 supports 4-bit priority level (highest 4-bits are implemented)

	NVIC_SetPriority (EXTI1_IRQn, 1);  // Set Priority
	NVIC_SetPriority (EXTI0_IRQn, 2);  // Set Priority

	NVIC_EnableIRQ (EXTI1_IRQn);  // Enable Interrupt
	NVIC_EnableIRQ (EXTI0_IRQn);  // Enable Interrupt
	// Finally we enable EXTI0 and EXTI1 from NVIC register
	// ISER[x] x={0..7} is written the interrupt number to enable that interrupt.
	// Interrupt numbers are preset. (6 for EXTI0 and 50 for TIM5)
	// ISER[0] holds the first 32 numbers, ISER[1] holds the next 32 numbers, ...
	// then the respective bit is enabled on ISER[x] by taking mod32 of the interrupt number.
	// Example:
	//  for EXTI0 we write ISER[0] = (1 << 6);
	//  for TIM5 we write ISER[1] = (1 << 18);
	// The enable_interrupt function does that automatically. Just pass the interrupt number to be enabled.
	//enable_interrupt(EXTI0_IRQn); // Enable EXTI0 interrupt on NVIC
	// or
	// NVIC->ISER[0] = (1 << 6);
	//enable_interrupt(EXTI1_IRQn); // Enable EXTI1 interrupt on NVIC

	}




/*
 * A generic external interrupt handler (EXTI2 example)
 */











