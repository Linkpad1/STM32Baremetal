/*
 * spi.h
 *
 *  Created on: Mar 10, 2024
 *      Author: Pee
 */

#ifndef SPI_H_
#define SPI_H_

#include "stm32f4xx.h"

void spi_innit(void);
void spi_write(uint8_t reg, uint8_t data);
uint8_t spi_read(uint8_t reg);

#endif /* SPI_H_ */
