/*
 * i2c.c
 *
 *  Created on: Mar 10, 2024
 *      Author: Pee
 */
// I2c for pins B6 & B9 (B6 = SCK | B9 = SDA)

#include "i2c.h"

// these should go to cs816t related headers
volatile uint8_t DeviceAddr = CS816T_ADDRESS;

void i2c_innit(void)
{
	//*******************************
	    // setup I2C - GPIOB 6, 7
	    //*******************************
	    // enable I2C clock
	    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

	    // setup I2C pins
	    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	    GPIOB->MODER &= ~(3U << 6*2); // PB6
	    GPIOB->MODER |=  (2 << 6*2); // AF
	    GPIOB->OTYPER |= (1 << 6);   // open-drain

	    GPIOB->MODER &= ~(3U << 7*2); // PB7
	    GPIOB->MODER |=  (2 << 7*2); // AF
	    GPIOB->OTYPER |= (1 << 7);   // open-drain

	    // choose AF4 for I2C1 in Alternate Function registers
	    GPIOB->AFR[0] |= (4 << 6*4);     // for pin 6

	    GPIOB->AFR[1] |= (4 << (7-8)*4); // for pin 7

	    // reset and clear reg
	    I2C1->CR1 = I2C_CR1_SWRST;
	    I2C1->CR1 = 0;

	    I2C1->CR2 |= (I2C_CR2_ITERREN); // enable error interrupt

	    // fPCLK1 must be at least 2 Mhz for SM mode
	    //        must be at least 4 Mhz for FM mode
	    //        must be multiple of 10Mhz to reach 400 kHz
	    // DAC works at 100 khz (SM mode)
	    // For SM Mode:
	    //    Thigh = CCR * TPCLK1
	    //    Tlow  = CCR * TPCLK1
	    // So to generate 100 kHz SCL frequency
	    // we need 1/100kz = 10us clock speed
	    // Thigh and Tlow needs to be 5us each
	    // Let's pick fPCLK1 = 10Mhz, TPCLK1 = 1/10Mhz = 100ns
	    // Thigh = CCR * TPCLK1 => 5us = CCR * 100ns
	    // CCR = 50
	    I2C1->CR2 |= (10 << 0); // 10Mhz periph clock
	    I2C1->CCR |= (50 << 0);
	    // Maximum rise time.
	    // Calculation is (maximum_rise_time / fPCLK1) + 1
	    // In SM mode maximum allowed SCL rise time is 1000ns
	    // For TPCLK1 = 100ns => (1000ns / 100ns) + 1= 10 + 1 = 11
	    I2C1->TRISE |= (11 << 0); // program TRISE to 11 for 100khz
	    // set own address to 00 - not really used in master mode
	    I2C1->OAR1 |= (0x00 << 1);
	    I2C1->OAR1 |= (1 << 14); // bit 14 should be kept at 1 according to the datasheet

	    // enable error interrupt from NVIC
	    NVIC_SetPriority(I2C1_ER_IRQn, 4);
	    NVIC_EnableIRQ(I2C1_ER_IRQn);

	    I2C1->CR1 |= I2C_CR1_PE; // enable i2c
}



void i2c_reset (void)
{
    //*******************************
    // setup reset pin for CS816t - GPIO A5
    //*******************************
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(3U << 5*2);
    GPIOA->MODER |=  (1 << 5*2);

    // activate CS816t
    GPIOA->ODR  |=  (1 << 5);
}


static inline void __i2c_start() {
    I2C1->CR1 |= I2C_CR1_START;
    while(!(I2C1->SR1 & I2C_SR1_SB));
}

static inline void __i2c_stop() {
    I2C1->CR1 |= I2C_CR1_STOP;
    while(!(I2C1->SR2 & I2C_SR2_BUSY));
}

void i2c_write(uint8_t regaddr, uint8_t data) {
    // send start condition
    __i2c_start();

    // send chipaddr in write mode
    // wait until address is sent
    I2C1->DR = DeviceAddr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    // dummy read to clear flags
    (void)I2C1->SR2; // clear addr condition

    // send MAP byte with auto increment off
    // wait until byte transfer complete (BTF)
    I2C1->DR = regaddr;
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    // send data
    // wait until byte transfer complete
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    // send stop condition
    __i2c_stop();
}

uint8_t i2c_read(uint8_t regaddr) {
    uint8_t reg;

    // send start condition
    __i2c_start();

    // send chipaddr in write mode
    // wait until address is sent
    I2C1->DR = DeviceAddr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    // dummy read to clear flags
    (void)I2C1->SR2; // clear addr condition

    // send MAP byte with auto increment off
    // wait until byte transfer complete (BTF)
    I2C1->DR = regaddr;
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    // restart transmission by sending stop & start
    __i2c_stop();
    __i2c_start();

    // send chipaddr in read mode. LSB is 1
    // wait until address is sent
    I2C1->DR = DeviceAddr | 0x01; // read
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    // dummy read to clear flags
    (void)I2C1->SR2; // clear addr condition

    // wait until receive buffer is not empty
    while (!(I2C1->SR1 & I2C_SR1_RXNE));
    // read content
    reg = (uint8_t)I2C1->DR;

    // send stop condition
    __i2c_stop();

    return reg;
}





