/*
 * cst816t.c
 * STM32 Baremetal CST816t driver
 *  Created on: Mar 18, 2024
 *      Author: Pee
 */

#include "cst816t.h"


uint8_t CST816T_Reset(void) {
    //HAL_GPIO_WritePin(CS816T_RST_PORT, CS816T_RST_PIN, GPIO_PIN_SET);
	DWT_Delay_us(50);
    //HAL_GPIO_WritePin(CS816T_RST_PORT, CS816T_RST_PIN, GPIO_PIN_RESET);
	DWT_Delay_us(5);
    //HAL_GPIO_WritePin(CS816T_RST_PORT, CS816T_RST_PIN, GPIO_PIN_SET);
	DWT_Delay_us(50);
}

void CST816T_WriteRegisters(uint8_t address, uint8_t *data, uint8_t size) {
	// I2C Send
	I2C1_Start();
	I2C1_SendAddress(CST816T_I2C_ADDR, I2C_TRANSMITTER);
	I2C1_SendData(address);

	for (int8_t i = 0; i < size; ++i) {
		I2C1_SendData(data[i]);
	}

	I2C1_Stop();
}

void CST816T_WriteRegister(uint8_t address, uint8_t data) {

	uint8_t tempdata[1] = { data };

	CST816T_WriteRegisters(address, tempdata, 1);
	}


void CST816T_ReadRegisters(uint8_t address, uint8_t *data, uint8_t size) {

	uint8_t count = 0;

	// I2C Send
	I2C1_Start();
	I2C1_SendAddress(CST816T_I2C_ADDR, I2C_TRANSMITTER);
	I2C1_SendData(address);

	// I2C Receive
	I2C1_Start();
	I2C1_SendAddress(CST816T_I2C_ADDR, I2C_RECEIVER);

	for (count = 0; count < size - 1; ++count ) {
		data[count] = I2C1_ReceiveData(I2C_ACK);
	}

	I2C1_Stop();
	data[count] = I2C1_ReceiveData(I2C_NACK);
}

uint8_t CST816T_ReadRegister(uint8_t address) {

	uint8_t data[1] = {0};

	 CST816T_ReadRegisters(address, data, 1);

	return data[0];
}

uint8_t CS816T_GetXY(void) {
	uint8_t data[3] = {0,0,0,0};
	CST816T_ReadRegisters(0x03,data, 4);
}

uint8_t CS816T_version(void) {
	return (CST816T_ReadRegister(CST816T_ReadXlo));
}

uint8_t CS816T_Gesture(void) {
	return (CS816T_GetRegByte(CST816T_GestureID));
}

uint8_t CS816T_Touch(void) {
	return (CS816T_GetRegByte(CST816T_ReadTouch));
}

void CS816T_GetTouchInfo(uint8_t   GestureID,
		                 uint8_t   ReadTouch,
						 uint16_t   X,
						 uint16_t   Y)
{
	uint8_t Xup = 0;
	uint8_t Xlo = 0;
	uint8_t Yup = 0;
	uint8_t Ylo = 0;
	uint8_t data[3] = {0,0,0,0};
	CST816T_ReadRegisters(0x03,data,4); // get x,y values (4 bytes)
	data[0] = Xup;
    data[1] = Xlo;
    data[2] = Yup;
    data[3] = Ylo;
	GestureID = CST816T_Gesture();
	ReadTouch = CST816T_Touch();
    // shift X&Y high bits 4

    X = ((Xup<<8)+Xlo);
    Y = ((Yup<<8)+Ylo);
    return GestureID;
    return ReadTouch;
    return X;
    return Y;
}

uint8_t CS816T_DecodeBCD_4(uint8_t bin) {
	return (((bin & 0xf0) >> 4) * 10) + (bin & 0x0f);
}



