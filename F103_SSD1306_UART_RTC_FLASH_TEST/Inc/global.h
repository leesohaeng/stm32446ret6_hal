/*
 * global.h
 *
 *  Created on: 2018. 11. 10.
 *      Author: TestParts
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

#include "main.h"
#include "stm32f1xx_hal.h"


typedef enum {FALSE = 0, TRUE = !FALSE} bool;
#define true TRUE
#define false FALSE


//   https://cafe.naver.com/circuitsmanual/200538
//   Á¦ÇÃ¸° ´ÔÀÇ µô·¹ÀÌ ÇÔ¼ö
#define delay_ms      HAL_Delay
#define millis()      HAL_GetTick()
#define SYS_CLOCK     72
#define SYSTICK_LOAD  71999

uint32_t micros();

void HAL_Delay(__IO uint32_t Delay);

void delay_us(uint32_t us);


typedef struct {
	uint8_t tCnt, t100ms, t500ms;
} TIM_User_InitTypeDef;



__STATIC_INLINE uint8_t day_of_week(uint16_t y, uint8_t m, uint8_t d)
{
	uint8_t t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
	y += 2000;
	y -= m < 3;
	return (y + y / 4 - y / 100 + y / 400 + t[m-1] + d) % 7;
}

#endif /* GLOBAL_H_ */
