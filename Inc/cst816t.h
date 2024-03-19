/*
 * cst816t.h
 *
 *  Created on: Mar 18, 2024
 *      Author: Pee
 */

#ifndef CST816T_H_
#define CST816T_H_

#include "i2c.h"
#include "stm32f4xx.h"
#include "timer.h"

#define CST816T_I2C_ADDR 	0x15

#define CST816T_GestureID   0x01
#define CST816T_ReadTouch   0x02
#define CST816T_ReadXup     0x03
#define CST816T_ReadXlo     0x04
#define CST816T_ReadYup     0x05
#define CST816T_ReadYlo     0x06
#define CST816T_ReadBPCup   0xB0
#define CST816T_ReadBPClo   0xB1

#define CST816T_ChipID      0xA7
#define CST816T_ProjID      0xA8
#define CST816T_FWVer       0xA9
#define CST816T_Sleep       0xFE

#define CST816T_TIMEOUT		1000


uint8_t CST816T_Reset(void);
void CST816T_WriteRegisters(uint8_t address, uint8_t *data, uint8_t size);
void CST816T_WriteRegister(uint8_t address, uint8_t data);
void CST816T_ReadRegisters(uint8_t address, uint8_t *data, uint8_t size);
uint8_t CST816T_ReadRegister(uint8_t address);

uint8_t CST816T_version(void);
uint8_t CST816T_Gesture(void);
uint8_t CST816T_Touch(void);
uint8_t CST816T_GetXY(void);

void CST816T_GetTouchInfo(uint8_t GestureID, uint8_t ReadTouch, uint16_t X, uint16_t Y);

uint8_t CS816T_DecodeBCD_4(uint8_t bin);




#endif /* CST816T_H_ */
