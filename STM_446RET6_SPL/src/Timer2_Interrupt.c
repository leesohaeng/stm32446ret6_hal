/*
 * timer2.c
 *
 *  Created on: Jan 13, 2016
 *      Author: sohae
 */

#include "Timer2_Interrupt.h"
#include <stdio.h>
uint16_t	ut = 0;

void	Timer2_Init(uint16_t Period)
{
	TIM_TimeBaseInitTypeDef TimerInitStructure;
	NVIC_InitTypeDef		nvicStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);

	// Set Timer Instructure
	// APB1은 180MHz의 반인 90MHz가 공급.
	// 90000000 / 6000 = 15000
	// System Clock / Prescaler 로 계산한다.
	// 1000 - 1 = 999
//	TimerInitStructure.TIM_Prescaler = 5999;
	TimerInitStructure.TIM_Prescaler = 8;
	TimerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;

	// 만약 PWM을 1KHz로 만들고 싶다면
	// 90000000 / 1000 = 90000
	// 90000 - 1 = 89999를 Period에 입력 하면 1초가 만들어짐.
//	TimerInitStructure.TIM_Period = (15*Period) - 1;
	TimerInitStructure.TIM_Period = (Period*10) - 1;  // 100만분의 1초.
	TimerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TimerInitStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM2, &TimerInitStructure );
	TIM_ITConfig(TIM2, TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM2, ENABLE);

	// Enable Timer Interrupt
	nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure.NVIC_IRQChannelSubPriority = 1;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&nvicStructure);

}

void	TIM2_IRQHandler()
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update)!=RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		GPIO_ToggleBits(GPIOA, GPIO_Pin_5);
	}
}
