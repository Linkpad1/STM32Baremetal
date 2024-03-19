/*
 * main.c
 *
 *  Created on: Jan 13, 2024
 *      Author: Patrick Lee - GPIO Baremetal implementation example for STM32F412G Discovery Board
 *
 *
 */



#include "stm32f4xx.h"
#include "gpio.h"
#include "usart.h"
#include "rcc.h"
#include "interrupt.h"
#include "timer_interrupt.h"
#include "i2c.h"



#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)    ((REG) & (BIT))


#define PIN0      (1U<<0)  //Define LED Pin0 (E0)
#define PIN1      (1U<<1)  //Define LED Pin0 (E1)
#define PIN2      (1U<<2)  //Define LED Pin0 (E3)
#define PIN3      (1U<<3)  //Define LED Pin0 (E4)

uint8_t flag1 = 0;
uint8_t flag2 = 0;
uint8_t flag3 = 0;

// External push button 1 interrupt
void EXTI0_IRQHandler(void) {

	if (EXTI->PR && EXTI_PR_PR0)    // If the PA1 triggered the interrupt
	{

		EXTI->PR |= EXTI_PR_PR0;  // Clear the interrupt flag by writing a 1

		  flag1 = 1;
	}
}

// External push button 2 interrupt
void EXTI1_IRQHandler(void) {

	if (EXTI->PR && EXTI_PR_PR1)    // If the PA1 triggered the interrupt
	{

	        EXTI->PR |= EXTI_PR_PR1;  // Clear the interrupt flag by writing a 1

	        flag2 = 1;
	}
}

// External timer interrupt
void TIM5_IRQHandler(void) {
  // Handle a timer 'update' interrupt event

	if (TIM5->SR && TIM_SR_UIF)    // If the PA1 triggered the interrupt
	{

		flag3 = 1;

		TIM5->SR &= ~TIM_SR_UIF;  // Clear the interrupt flag by writing a 1

      }
}


int main(void)
{

	SetSystemClockTo16Mhz();


	UART2_innit();
	I2C1_Init(); // PB6 & PB7
	configureTimer2();

	interrupt_setup();
	DWT_Delay_Init(); // delay using system clock

	ConfigureTimer5();



	while(1)
	{

/*

*/
		if (flag1 ==1)
				{
			UART2_send("Interrupt1",sizeof"Interrupt1");

					flag1 = 0;
				}

		if (flag2 ==1)
					{
				UART2_send("Interrupt2",sizeof"Interrupt2");

						flag2 = 0;
					}

		if (flag3 ==1)
						{
				UART2_send("Interrupt3",sizeof"Interrupt3");

							flag3 = 0;
						}



		delay( 10 ); // 1 Second Delay
		UART2_send("testing 123",sizeof"testing 123");


}

}






