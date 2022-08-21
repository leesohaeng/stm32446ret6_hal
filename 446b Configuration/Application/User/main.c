/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
#define DATA  80

uint8_t Rx_Buffer1[DATA];
uint8_t Rx_Buffer2[DATA];
uint8_t Rx_Buffer3[DATA];

uint8_t Rx_Data1[2];
uint8_t Rx_Data2[2];
uint8_t Rx_Data3[2];

uint8_t Rx_Indx1;
uint8_t Rx_Indx2;
uint8_t Rx_Indx3;

uint8_t Transfer_cplt1;
uint8_t Transfer_cplt2;
uint8_t Transfer_cplt3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void 		SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
void 		HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void 		UART_Putc(UART_HandleTypeDef *huart,char c);
void 		UART_Puts(UART_HandleTypeDef *huart,const char *s);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{
	char		s[80];
	uint32_t	t;
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

  /* Configure the system clock */
	SystemClock_Config();

  /* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();

	if(HAL_RCC_GetHCLKFreq()==180000000)
	{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,SET);
	}

  /* USER CODE BEGIN 2 */
	// Activate uart rx interrupt every time receiving 1 byte
	while(HAL_UART_Receive_IT(&huart1, Rx_Data1, 1)!=HAL_OK);
	while(HAL_UART_Receive_IT(&huart2, Rx_Data2, 1)!=HAL_OK);
	while(HAL_UART_Receive_IT(&huart3, Rx_Data3, 1)!=HAL_OK);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	t = 0;
	while (1)
	{
		sprintf(s,"*Sin %4lu : %10.5lf\r\n",t,sin(t*3.141592/180));
		HAL_UART_Transmit(&huart1, (uint8_t *)s, strlen(s),1000);
		sprintf(s,"*Cos %4lu : %10.5lf\r\n",t,cos(t*3.141592/180));
		HAL_UART_Transmit(&huart2, (uint8_t *)s, strlen(s),1000);
		sprintf(s,"*Tan %4lu : %10.5lf\r\n",t,tan(t*3.141592/180));
		HAL_UART_Transmit(&huart3, (uint8_t *)s, strlen(s),1000);

		if(Transfer_cplt1)
		{
			HAL_UART_Transmit(&huart1, Rx_Buffer1, sizeof(Rx_Buffer1),1000);
			HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2,1000);
			Transfer_cplt1 = 0;
		}
		if(Transfer_cplt2)
		{
			HAL_UART_Transmit(&huart2, Rx_Buffer2, sizeof(Rx_Buffer2),1000);
			HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2,1000);
			Transfer_cplt2 = 0;
		}
		if(Transfer_cplt3)
		{
			HAL_UART_Transmit(&huart3, Rx_Buffer3, sizeof(Rx_Buffer3),1000);
			HAL_UART_Transmit(&huart3, (uint8_t *)"\r\n", 2,1000);
			Transfer_cplt3 = 0;
		}
		t++;
		if(t>90) t=0;
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
		// HAL_Delay(10000);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  HAL_PWREx_ActivateOverDrive();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

}

/* USART3 init function */
void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart3);

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t	i;

	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

	if(huart->Instance==USART1)
	{
		if(Rx_Indx1==0)
		{
			for(i=0;i<DATA;i++) Rx_Buffer1[i] = 0;
		}
		if(Rx_Data1[0]!=0x0d)
		{
			Rx_Buffer1[Rx_Indx1++]=Rx_Data1[0];
		}
		else
		{
			Rx_Indx1 = 0;
			Transfer_cplt1 = 1;
		}
		HAL_UART_Receive_IT(&huart1, Rx_Data1, 1);
		HAL_UART_Transmit(&huart1, Rx_Data1, 1,1000);
	}
	else if(huart->Instance==USART2)
	{
		if(Rx_Indx2==0)
		{
			for(i=0;i<DATA;i++) Rx_Buffer2[i] = 0;
		}
		if(Rx_Data2[0]!=0x0d)
		{
			Rx_Buffer2[Rx_Indx2++]=Rx_Data2[0];
		}
		else
		{
			Rx_Indx2 = 0;
			Transfer_cplt2 = 1;
		}
		HAL_UART_Receive_IT(&huart2, Rx_Data2, 1);
		HAL_UART_Transmit(&huart2, Rx_Data2, 1,1000);
	}
	else if(huart->Instance==USART3)
	{
		if(Rx_Indx3==0)
		{
			for(i=0;i<DATA;i++) Rx_Buffer3[i] = 0;
		}
		if(Rx_Data3[0]!=0x0d)
		{
			Rx_Buffer3[Rx_Indx3++]=Rx_Data3[0];
		}
		else
		{
			Rx_Indx3 = 0;
			Transfer_cplt3 = 1;
		}
		HAL_UART_Receive_IT(&huart3, Rx_Data3, 1);
		HAL_UART_Transmit(&huart3, Rx_Data3, 1,1000);
	}
}
/* USER CODE END 4 */

void UART_Putc(UART_HandleTypeDef *huart,char c)
{
	while(!(huart->Instance->SR & UART_FLAG_TXE));
	huart->Instance->DR = c;
}

void UART_Puts(UART_HandleTypeDef *huart,const char *s)
{
	while(*s) {
		UART_Putc(huart,*s);
		s++;
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
