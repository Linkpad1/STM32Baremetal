/*
 * spi.h
 *
 *  Created on: Mar 10, 2024
 *      Author: Pee
 */

#ifndef SPI_H_
#define SPI_H_

#include "stm32f4xx.h"
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

void SPI1_Init();
void SPI1_EnableSlave();
void SPI1_DisableSlave();
uint16_t SPI1_Write(uint16_t data);


#endif /* SPI_H_ */
