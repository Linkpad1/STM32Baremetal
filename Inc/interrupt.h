/*
 * interrupt.h
 *
 *  Created on: Mar 4, 2024
 *      Author: Pee
 */

#ifndef INTERRUPT_H_
#define INTERRUPT_H_

#include "main.h"
#include "stm32f4xx.h"



#define int32_t         int
#define int16_t         short
#define int8_t          char
#define uint32_t        unsigned int
#define uint16_t        unsigned short
#define uint8_t         unsigned char

#define STACKINIT       0x20008000

typedef struct
{
	uint32_t   ISER[8];     /* Address offset: 0x000 - 0x01C */
	uint32_t  RES0[24];     /* Address offset: 0x020 - 0x07C */
	uint32_t   ICER[8];     /* Address offset: 0x080 - 0x09C */
	uint32_t  RES1[24];     /* Address offset: 0x0A0 - 0x0FC */
	uint32_t   ISPR[8];     /* Address offset: 0x100 - 0x11C */
	uint32_t  RES2[24];     /* Address offset: 0x120 - 0x17C */
	uint32_t   ICPR[8];     /* Address offset: 0x180 - 0x19C */
	uint32_t  RES3[24];     /* Address offset: 0x1A0 - 0x1FC */
	uint32_t   IABR[8];     /* Address offset: 0x200 - 0x21C */
	uint32_t  RES4[56];     /* Address offset: 0x220 - 0x2FC */
	uint8_t   IPR[240];     /* Address offset: 0x300 - 0x3EC */
	uint32_t RES5[644];     /* Address offset: 0x3F0 - 0xEFC */
	uint32_t       STIR;    /* Address offset:         0xF00 */
} NVIC_type;

typedef struct
{
	uint32_t IMR;   /* Interrupt mask register,            Address offset: 0x00 */
	uint32_t EMR;   /* Event mask register,                Address offset: 0x04 */
	uint32_t RTSR;  /* Rising trigger selection register,  Address offset: 0x08 */
	uint32_t FTSR;  /* Falling trigger selection register, Address offset: 0x0C */
	uint32_t SWIER; /* Software interrupt event register,  Address offset: 0x10 */
	uint32_t PR;    /* Pending register,                   Address offset: 0x14 */
} EXTI_type;



void enable_interrupt(IRQn_Type IRQn);
void disable_interrupt(IRQn_Type IRQn);

void button_handler(void);
void interrupt_setup(void);




/*************************************************
* Vector Table
*************************************************/



#endif /* INTERRUPT_H_ */
