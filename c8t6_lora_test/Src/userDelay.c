/*
 * userDelay.c
 *
 *  Created on: 2016. 11. 26.
 *      Author: sohaeng lee
 *      446RE에서는 NOP횟수는 44로 설정.
 */
#include "userDelay.h"

void delay_us(uint16_t count)
{
	int n;
    // 446re에서는 n=44
	// c8t6dptjsms n=7
	for(;count!=0;count--)
	for(n=0;n<7;n++) asm volatile("NOP");
}

void delay_ms(uint16_t count)
{
	for(;count!=0;count--) delay_us(1000);
}
