/*
 * global.c
 *
 *  Created on: 2018. 11. 10.
 *      Author: TestParts
 */

#include "global.h"

extern __IO uint32_t uwTick;

uint32_t micros()
{
	return (uwTick&0x3FFFFF)*1000 + (SYSTICK_LOAD-SysTick->VAL)/SYS_CLOCK;
}

void HAL_Delay(__IO uint32_t Delay)
{
	uint32_t tickstart = HAL_GetTick();

	while((millis() - tickstart) < Delay);
}

void delay_us(uint32_t us)
{
	uint32_t temp = micros();
	uint32_t comp = temp + us;
	uint8_t  flag = 0;
	while(comp > temp)
	{
		if(((uwTick&0x3FFFFF)==0)&&(flag==0))
		{
			flag = 1;
		}
		if(flag) temp = micros() + 0x400000UL * 1000;
		else     temp = micros();
	}
}

