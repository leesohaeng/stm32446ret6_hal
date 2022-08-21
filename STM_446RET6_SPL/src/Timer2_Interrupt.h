/*
 * timer2.h
 *
 *  Created on: Jan 13, 2016
 *      Author: sohae
 */

#ifndef TIMER2_H_
#define TIMER2_H_

#endif /* TIMER2_H_ */

#include "stm32f4xx_tim.h"
#include "main.h"

void	Timer2_Init(uint16_t Period);
void	TIM2_IRQHandler();
