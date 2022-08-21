/*
 * tlcd.c
 *
 *  Created on: Jan 5, 2016
 *      Author: LEE SO HAENG
 */
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "tlcd.h"
#include "stm32f4xx_it.h"

uint32_t TRS, TEN,  TData4, TData5, TData6, TData7;
GPIO_TypeDef *GPIOx;
// TLCD를 사용하기 위해 GPIOx 10,11.........0,1,2,3
// 0->RS, 1->E
// SYSTICK_Config는 반드시 1000000으로 설정할것~!!!!!
void	TLCD_Init(GPIO_TypeDef *GPIOn, uint32_t RS,uint32_t EN, uint32_t Data4,uint32_t Data5,uint32_t Data6,uint32_t Data7)
{
	GPIO_InitTypeDef	GPIO_InitStruct;

	GPIOx = GPIOn;
	TRS = RS;
	TEN = EN;
	TData4 = Data4;
	TData5 = Data5;
	TData6 = Data6;
	TData7 = Data7;

	GPIO_InitStruct.Pin = RS | EN | Data4 | Data5 | Data6 | Data7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOn, &GPIO_InitStruct);

	// GPIOA 0,1 --> Control pin
	// GPIOA 2,3,4,5 --> Data pin
	HAL_GPIO_WritePin(GPIOn , RS | EN | Data4 | Data5 | Data6 | Data7 , GPIO_PIN_RESET);
	HAL_Delay(10000);  // 10ms

	TLCD_4BitWrite(0x03);
	HAL_Delay(5000);   // 5ms
	TLCD_4BitWrite(0x03);
	HAL_Delay(5000);   // 5ms
	TLCD_4BitWrite(0x03);
	HAL_Delay(5000);   // 5ms
	TLCD_4BitWrite(0x02);
	HAL_Delay(5000);   // 5ms

	// Function Set
	// DL : 0->4bit, 1->8bit
	// N : 0->1 line, 1->2 line
	// F : 0-> 5*7, 1->5*10
	// I selected 4bit, 2 line, 5*7
	TLCD_Cmd( 0x28 );
	HAL_Delay(40);  // 40us

	// Display Set
	// D : 0->disp off, 1->disp on
	// C : 0->Cur off, 1->Cur on
	// B : 0->Cur blink off, 1->Cur blink on
	TLCD_Cmd( 0x0C );
	HAL_Delay(40);  // 40us

	// Display Mode
	// I/D : 0->Dec Cursor pos, 1->Inc Cursor pos
	// S : 0-> No disp shift, 1-> Disp shift
	TLCD_Cmd( 0x06 );
	HAL_Delay(40);  // 40us

	// LCD Clear display
	TLCD_Cmd( 0x01 );
	HAL_Delay(2000);  // 2ms
}

void	TLCD_4BitWrite(BYTE c)
{
	if( c & 0x01 ) 	HAL_GPIO_WritePin(GPIOx, TData4 , GPIO_PIN_SET);
	else			HAL_GPIO_WritePin(GPIOx, TData4 , GPIO_PIN_RESET);

	if( c & 0x02 ) 	HAL_GPIO_WritePin(GPIOx, TData5 , GPIO_PIN_SET);
	else			HAL_GPIO_WritePin(GPIOx, TData5 , GPIO_PIN_RESET);

	if( c & 0x04 ) 	HAL_GPIO_WritePin(GPIOx, TData6 , GPIO_PIN_SET);
	else			HAL_GPIO_WritePin(GPIOx, TData6 , GPIO_PIN_RESET);

	if( c & 0x08 ) 	HAL_GPIO_WritePin(GPIOx, TData7 , GPIO_PIN_SET);
	else			HAL_GPIO_WritePin(GPIOx, TData7 , GPIO_PIN_RESET);
	TLCD_Enable();
}

void	TLCD_Enable()
{
	HAL_GPIO_WritePin(GPIOx, TEN , GPIO_PIN_SET);
	HAL_Delay(2);
	HAL_GPIO_WritePin(GPIOx, TEN , GPIO_PIN_SET);
	HAL_Delay(2);
	HAL_GPIO_WritePin(GPIOx, TEN , GPIO_PIN_RESET);
	HAL_Delay(40);
}

void 	TLCD_Cmd(BYTE cmd)
{
	TNLD_Send(cmd,LOW);
}

void	TLCD_Write(BYTE v)
{
	TNLD_Send(v,HIGH);
}

void	TNLD_Send(uint8_t v,uint8_t m)
{
	if( m ) 	HAL_GPIO_WritePin(GPIOx, TRS , GPIO_PIN_SET);
	else		HAL_GPIO_WritePin(GPIOx, TRS , GPIO_PIN_RESET);

	TLCD_4BitWrite(v>>4);
	TLCD_4BitWrite(v);
}

void	TLCD_Putc(char c)
{
	TLCD_Write((BYTE)c);
	HAL_Delay(1);
}

void	TLCD_Puts(uint8_t line, uint8_t n,char *s)
{
	if(line==1)		TLCD_Cmd(0x80+n);
	else 			TLCD_Cmd(0xC0+n);

	while(*s) {
		TLCD_Putc(*s);
		s++;
	}
}
