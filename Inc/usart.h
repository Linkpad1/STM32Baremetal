/*
 * usart.h
 *
 *  Created on: Feb 4, 2024
 *      Author: Pee
 */

#include <stddef.h>

#ifndef USART_H_
#define USART_H_


void UART2_innit(void);

void delay(int n);

void UART2_send(char *msg, size_t len);

#endif /* USART_H_ */
