/*
 * i2c.c
 *
 *  Created on: Mar 10, 2024
 *      Author: Pee
 */
// I2c for pins B6 & B9 (B6 = SCK | B9 = SDA)

#include "i2c.h"

//I2C1
//PB6 = SCL
//PB7 = SDA


// these should go to cs816t related headers
void I2C1_Init(void) {

	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;				// 1: I2C1 clock enabled
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;			// 1: I/O port B clock enabled

	// PB6 I2C1
    GPIOB->MODER |=  (2 << 6*2);    // AF6
    GPIOB->OTYPER |= (1 << 6);      // open-drain
    GPIOB->OSPEEDR = (3 << 2*6);  	// OSPEEDR6[1:0] = 11 => High speed
    GPIOB->PUPDR = (1 << 2*6 );     // Pull up Register
    GPIOB->AFR[0] |= (4 << 24);     // for pin 6 [AF4]

	// PB7 I2C1
    GPIOB->MODER |=  (2 << 7*2);     // AF7
    GPIOB->OTYPER |= (1 << 7);      // open-drain
    GPIOB->OSPEEDR = (3 << 2*7);  	// OSPEEDR6[1:0] = 11 => High speed
    GPIOB->PUPDR = (1 << 2*7);      // Pull up Register
    GPIOB->AFR[0] |= (4 << 28);     // for pin 7 [AF4]

    // reset and clear reg
	I2C1->CR1 = I2C_CR1_SWRST;
	I2C1->CR1 = 0;

	I2C1->CR2 |= (I2C_CR2_ITERREN); // enable error interrupt

	I2C1->CR2 = 16;		             // 16Mhz periph clock
	I2C1->CCR = 80;				    // F/S (Bit 15) = 0 => standard master mode
									// CCR[11:0] (Bits 11:0) =  80 => SCL clock = 100kHz
    // Maximum rise time.
    // Calculation is (maximum_rise_time / fPCLK1) + 1
    // In SM mode maximum allowed SCL rise time is 1000ns
    // For TPCLK1 = 100ns => (1000ns / 100ns) + 1= 10 + 1 = 11
    I2C1->TRISE |= (11 << 0); // program TRISE to 11 for 100khz
    // set own address to 00 - not really used in master mode
    I2C1->OAR1 |= (0x00 << 1);
    I2C1->OAR1 |= (1 << 14); // bit 14 should be kept at 1 according to the datasheet

    // enable error interrupt from NVIC
    //NVIC_SetPriority(I2C1_ER_IRQn, 4);
    //NVIC_EnableIRQ(I2C1_ER_IRQn);

    I2C1->CR1 |= I2C_CR1_PE; // enable i
}


void I2C1_Start() {

	//while ((I2C1->SR2 & I2C_SR2_BUSY));			// 1: Communication ongoing on the bus

	I2C1->CR1 |= I2C_CR1_START;						// 1: Repeated start generation

	while ( !(I2C1->SR1 & I2C_SR1_SB)  ||			// 0: No Start condition
			!(I2C1->SR2 & I2C_SR2_MSL) ||			// 0: Slave Mode
			!(I2C1->SR2 & I2C_SR2_BUSY)				// 0: No communication on the bus
	);
}


void I2C1_Stop() {

	I2C1->CR1 |= I2C_CR1_STOP;						// 1: Stop generation after the current byte transfer or after the current Start condition is sent.

	while((I2C1->SR1 & I2C_SR1_STOPF));				// 1: Stop condition detected
}


I2C_Status_Type I2C1_SendAddress(uint8_t address, I2C_Direction_Type direction) {

	uint32_t timeout = 1000000;

	address <<= 1;

	if (direction == I2C_TRANSMITTER) {
		address &= ~(1 << 0);						// Reset the address bit0 for write
		I2C1->DR = address;
		while ((!(I2C1->SR1 & I2C_SR1_ADDR) ||		// 0: No end of address transmission
				!(I2C1->SR2 & I2C_SR2_MSL)  ||		// 0: Slave Mode
				!(I2C1->SR2 & I2C_SR2_BUSY) ||		// 0: No communication on the bus
				!(I2C1->SR2 & I2C_SR2_TRA)) &&		// 0: Data bytes received
				--timeout
		);
	} else if (direction == I2C_RECEIVER) {
		address |= (1 << 0);						// Set the address bit0 for read
		I2C1->DR = address;
		while ((!(I2C1->SR1 & I2C_SR1_ADDR) ||		// 0: No end of address transmission
				!(I2C1->SR2 & I2C_SR2_MSL)  ||		// 0: Slave Mode
				!(I2C1->SR2 & I2C_SR2_BUSY))&&		// 0: No communication on the bus
				--timeout
		);
	}

	if (timeout <= 0) {
		return I2C_ERROR;
	}

	return I2C_OK;
}

void I2C1_SendData(uint8_t data) {

	I2C1->DR = data;

	while ( !(I2C1->SR1 & I2C_SR1_BTF)  ||			// 0: Data byte transfer not done
			!(I2C1->SR1 & I2C_SR1_TXE)  ||			// 0: Data register not empty
			!(I2C1->SR2 & I2C_SR2_MSL)  ||			// 0: Slave Mode
			!(I2C1->SR2 & I2C_SR2_BUSY) ||			// 0: No communication on the bus
			!(I2C1->SR2 & I2C_SR2_TRA)				// 0: Data bytes received
	);
}

uint8_t I2C1_ReceiveData(I2C_Acknowledge_Type acknowledge) {

	if (acknowledge == I2C_ACK) {
		I2C1->CR1 |= I2C_CR1_ACK;					// 1: Acknowledge returned after a byte is received (matched address or data)
	} else if (acknowledge == I2C_NACK) {
		I2C1->CR1 &= ~I2C_CR1_ACK;					// 0: No acknowledge returned
	}

	while ( !(I2C1->SR1 & I2C_SR1_RXNE) ||			// 0: Data register empty
			!(I2C1->SR2 & I2C_SR2_MSL)  ||			// 0: Slave Mode
			!(I2C1->SR2 & I2C_SR2_BUSY)				// 0: No communication on the bus
	);

	return (uint8_t)I2C1->DR;
}











