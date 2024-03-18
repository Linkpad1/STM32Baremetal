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

//typedef serial.baudrate = 9600;


#define PIN0      (1U<<0)  //Define LED Pin0 (E0)
#define PIN1      (1U<<1)  //Define LED Pin0 (E1)
#define PIN2      (1U<<2)  //Define LED Pin0 (E3)
#define PIN3      (1U<<3)  //Define LED Pin0 (E4)

uint8_t flag1 = 0;
uint8_t flag2 = 0;
uint8_t flag3 = 0;


void EXTI0_IRQHandler(void) {

	if (EXTI->PR && EXTI_PR_PR0)    // If the PA1 triggered the interrupt
	{

		EXTI->PR |= EXTI_PR_PR0;  // Clear the interrupt flag by writing a 1

		  flag1 = 1;
	}
}

void EXTI1_IRQHandler(void) {

	if (EXTI->PR && EXTI_PR_PR1)    // If the PA1 triggered the interrupt
	{

	        EXTI->PR |= EXTI_PR_PR1;  // Clear the interrupt flag by writing a 1

	        flag2 = 1;
	}
}

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

	//GPIOEoutputLED();

	//CLOCK_Init();
     i2c_reset(); // Pin D4 reset pin

	UART2_innit();
	i2c_innit();
	configureTimer2();

	interrupt_setup();

	//timer interrupt setup
	ConfigureTimer5();
	//start_timer(TIM4,999);
	//enable_Timer_4();


	while(1)
	{

		//GPIOE->ODR ^= PIN0; // toggle Pin 0 output
		//GPIOE->ODR ^= PIN1; // toggle Pin 1 output
		//GPIOE->ODR ^= PIN2; // toggle Pin 2 output
		//GPIOE->ODR ^= PIN3; // toggle Pin 3 output
/*
		GPIOE->BSRR = 0x00000001; // set Pin 0 output
		GPIOE->BSRR = 0x00000010; // set Pin 1 output
		GPIOE->BSRR = 0x00000100; // set Pin 2 output
		GPIOE->BSRR = 0x00001000; // set Pin 3 output
		GPIOE->BSRR = 0x00000001; // set Pin 0 output
		GPIOE->BSRR = 0x00010000; // reset Pin 0 output
		GPIOE->BSRR = 0x00100010; // reset Pin 1 output
		GPIOE->BSRR = 0x01000100; // reset Pin 2 output
		GPIOE->BSRR = 0x10001000; // reset Pin 3 output
*/

//		for(int i=0;i<1000000;i++){}



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

		//uart_write_buf(buffer,sizeof(buffer));
		//uart_write_byte(c);
		//UART2_SendChar ("C");
		//uart_write_byte(c);

		//uart_write_byte(a);

		delay( 10 ); // 1 Second Delay
		UART2_send("testing 123",sizeof"testing 123");
	    // read Chip ID - first 5 bits of CHIP_ID_ADDR
		char buffer[8] = {0};
	    uint8_t ret = i2c_read(CS816T_ChipID);
        //sprintf(buffer,strlen(buffer), "%u \n", ret);
	    //UART2_send((buffer),sizeof(buffer));

}

}






