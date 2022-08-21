/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/main.c 
  * @author  MCD Application Team
  * @version V1.6.1
  * @date    21-October-2015
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Timer2_Interrupt.h"
#include "arm_math.h"
#include <math.h>

/** @addtogroup Template_Project
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t uwTimingDelay;
RCC_ClocksTypeDef RCC_Clocks;

/* Private function prototypes -----------------------------------------------*/
//static void Delay(__IO uint32_t nTime);
void 		Delay(__IO uint32_t nCount);
void 		mDelay();
void 		GPIOA_Init(void);
void 		GPIOC_Init(void);

/* Private functions ---------------------------------------------------------*/
void Delay(__IO uint32_t nCount)
{
       /* Decrement nCountvalue */
       while (nCount != 0) {
              nCount--;
       }
}

void 		mDelay()
{
	__IO uint8_t t;
//	double       d;

	for(t=1;t<=100;t++) arm_sin_f32(t);
}

void GPIOA_Init(void)                    // PA0  Input for User Button
{ 					                    // ResetClockControl RCC
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void GPIOC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{

  SystemInit();
  RCC_GetClocksFreq(&RCC_Clocks);
  while(SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000000));
  /* Add your application code here */
  GPIOA_Init();
  GPIOC_Init();
//  Timer2_Init(1);
  /* HSE clock selected to output on MCO1 pin(PA8)*/
//  RCC_MCO1Config(RCC_MCO1Source_HSE, RCC_MCO1Div_1);
  
  /* Output SYSCLK/4 clock on MCO2 pin(PC9) ***********************************/ 
  /* Enable the GPIOACperipheral */ 
    
  /* SYSCLK/4 clock selected to output on MCO2 pin(PC9)*/
//  RCC_MCO2Config(RCC_MCO2Source_SYSCLK, RCC_MCO2Div_4);

  /* Infinite loop */
  while (1)
  {
/*	  a++;
	  GPIO_ToggleBits( GPIOA, GPIO_Pin_5 );
	  GPIO_ToggleBits( GPIOC, GPIO_Pin_8 );
	  Delay(1000);
	  if(a>100) a = 0; */

	  GPIOA->BSRRL = GPIO_Pin_5;
	  mDelay();
	  GPIOA->BSRRH = GPIO_Pin_5;
	  mDelay();
  }
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
/*
void Delay(__IO uint32_t nTime)
{
  uwTimingDelay = nTime;

  while(uwTimingDelay != 0);
} */

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (uwTimingDelay != 0x00)
  { 
    uwTimingDelay--;
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
