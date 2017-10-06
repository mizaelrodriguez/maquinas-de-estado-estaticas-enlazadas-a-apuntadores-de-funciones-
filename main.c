/*
 * Copyright (c) 2016, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    Practica1.c
 * @brief   Application entry point.
 */
#include "MK64F12.h"
#include "DataTypeDefinitions.h"
#include "GPIO.h"


void apagar();
void verde();
void azul();
void morado();
void amarillo();
void rojo();
void init_t();


typedef struct
{
	uint32 out;
	void(*fptrPort)(uint32);
	uint16 wait;
	void (*fptrDelay)(uint16);
	uint8 next[2];
}StateType;



StateType Sequence1[2]=
		{
			{LED_GREEN_ON,		ONESECOND, ON},//MAQUIN DE ESTADOS PARA SW3
			{LED_GREEN_OFF, 	ONESECOND, OFF},
			{LED_BLUE_ON, 		ONESECOND, ON},
			{LED_BLUE_OFF, 		ONESECOND, OFF},
			{LED_PURPLE_ON, 	ONESECOND, ON},
			{LED_PURPLE_OFF, 	ONESECOND, OFF},
			{LED_RED_ON, 		ONESECOND, ON},
			{LED_RED_ON, 		ONESECOND, ON},
			{LED_HELLOW_ON, 	ONESECOND, OFF},
			{LED_HELLOW_OFF, 	ONESECOND, ON},
		};

StateType Sequence2[2]=
		{
			{LED_HELLOW_ON, 	ONESECOND, ON},//MAQUINA DE ESTADOS PARA SW2
			{LED_HELLOW_OFF, 	ONESECOND, OFF},
			{LED_RED_ON, 		ONESECOND, ON},
			{LED_RED_ON, 		ONESECOND, OFF},
			{LED_PURPLE_ON, 	ONESECOND, ON},
			{LED_PURPLE_OFF, 	ONESECOND, OFF},
			{LED_BLUE_ON, 		ONESECOND, ON},
			{LED_BLUE_OFF, 		ONESECOND, OFF},
			{LED_GREEN_ON,		ONESECOND, ON},
			{LED_GREEN_OFF, 	ONESECOND, OFF},




		};


int main(void) {
	uint8 currentState = ON; //variables para la maquina de estados (sw3)
	uint32 output=0;
	uint32 input=0;

	uint8 currentState1 = ON; //variables para la maquina de estados (sw2)
	uint32 output1=0;
	uint32 input1=0;


	for(;;)
	{
		output = FSM_Moore[currentState].out;

	}
}

void verde(){
	apagar();
	GPIOE->PCOR = 0x4000000;/**Green led on*/
	psleep(7);
}

void azul(){
	apagar();
	GPIOB->PCOR = 0x00200000;/**Blue led on*/
}

void morado(){
	apagar();
	GPIOB->PCOR = 0x00200000;/**Blue led on*/
	GPIOB->PCOR = 0x00400000;/**Red led on*/
	psleep(7);
}

void amarillo(){
	apagar();
	GPIOE->PCOR = 0x4000000;/**Green led on*/
	GPIOB->PCOR = 0x00400000;/**Red led on*/
}

void rojo(){
	apagar();
	GPIOB->PCOR = 0x00400000;/**Red led on*/
}

void apagar(){
	GPIOB->PSOR = 0x00400000;/**Red led off*/
	GPIOE->PSOR = 0x4000000;/**Green led off*/
	GPIOB->PSOR = 0x00200000;/**Blue led off*/
}

void init_t(){

	//PORT A, B, C, E
	SIM->SCGC5 = 0x2E00;

	//GPIO PORT A, B, C, E
	PORTA->PCR[4] = PORT_PCR_MUX(1)|PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	PORTB->PCR[21] = PORT_PCR_MUX(1);
	PORTB->PCR[22] = PORT_PCR_MUX(1);
	PORTC->PCR[6] = PORT_PCR_MUX(1)|PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	PORTE->PCR[26] = PORT_PCR_MUX(1);

	//SAFE VALUE FOR GPIO PORTS
	GPIOA->PSOR = 0x10;
	GPIOB->PSOR = 0x600000;
	GPIOC->PSOR =0x40;
	GPIOE->PSOR = 0x4000000;

	//CONFIGURING I/O PINS

	//PB AS AN INPUT
	GPIOC->PDDR &= ~(0x40);
	GPIOA->PDDR &= ~(0x10);
	//LEDS AS OUTPUT
	GPIOB->PDDR = 0x00600000;
	GPIOE->PDDR = 0x04000000;

}

