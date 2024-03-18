/*
 * i2c.h
 *
 *  Created on: Mar 10, 2024
 *      Author: Pee
 */

#ifndef I2C_H_
#define I2C_H_

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

#define CS816T_ADDRESS 0x15
#define CS816T_REG_ID  0x00
#define CS816T_CHIP_ID 0x1C // first 5 bits of reg


#define CS816T_ChipID 0xA7
#define CS816T_FWver  0xA9






void i2c1_innit(void);
void i2c_reset (void);
void i2c_write(uint8_t regaddr, uint8_t data);
uint8_t i2c_read(uint8_t regaddr);



#endif /* I2C_H_ */
