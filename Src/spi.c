/*
 * spi.c
 *
 *  Created on: Mar 10, 2024
 *      Author: Pee
 */
// SPI for pins A5,A6,A7



#include "spi.h"

/*
 * SPI1
 * SCK : PA5 (Alternate function push-pull)
 * MISO: PA6 (Input floating / Input pull-up)
 * MOSI: PA7 (Alternate function push-pull)
 *       PA9 (Slave Select)
  */

void SPI1_Init() {

	RCC->APB1ENR |= RCC_APB2ENR_SPI1EN;				// 1: SPI1 clock enabled
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;			// 1: I/O port A clock enabled

	// PA5 SPI1
	GPIOA->MODER |= (2<<10);
    GPIOA->OSPEEDR |= (3<<10);
    GPIOA->AFR[0] |= (5<<20); //AF5(SPI1)

	// PA6 SPI1
	GPIOA->MODER |= (2<<12);
    GPIOA->OSPEEDR |= (3<<12);
    GPIOA->AFR[0] |= (5<<24); //AF5(SPI1)

	// PA7 SPI1
	GPIOA->MODER |= (2<<14);
    GPIOA->OSPEEDR |= (3<<14);
    GPIOA->AFR[0] |= (5<<28); //AF5(SPI1)

	// PA9 SPI1
	GPIOA->MODER |= (1<<18);
    GPIOA->OSPEEDR |= (3<<18);

    // SPI1
	SPI1->CR1 = 0;								// Reset
	SPI1->CR1 |= SPI_CR1_MSTR;					// 1: Master configuration
	SPI1->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1;	// 110: fPCLK/128
	SPI1->CR1 |= SPI_CR1_SSI;					// SSI: Internal slave select
	SPI1->CR1 |= SPI_CR1_SSM;					// 1: Software slave management enabled

	#ifdef SPI1_16_BIT_FORMAT
	SPI1->CR1 |= SPI_CR1_DFF;					// 1: 16-bit data frame format is selected for transmission/reception
	#endif

	SPI1->CR1 |= SPI_CR1_SPE;					// 1: Peripheral enabled

	SPI1_DisableSlave();
}

void SPI1_EnableSlave() {

	SPI1->CR1 |= (1<<6);   // SPE=1, Peripheral enabled
}

void SPI1_DisableSlave() {

	SPI1->CR1 &= ~(1<<6);   // SPE=0, Peripheral Disabled
}

uint16_t SPI1_Write(uint16_t data) {

	SPI1->DR = data;

	while (!(SPI1->SR & SPI_SR_TXE));			// 0: Tx buffer not empty
	while (!(SPI1->SR & SPI_SR_RXNE));			// 0: Rx buffer empty
	while (SPI1->SR & SPI_SR_BSY);				// 1: SPI (or I2S) is busy in communication or Tx buffer is not empty

	return (uint16_t)SPI1->DR;
}

























