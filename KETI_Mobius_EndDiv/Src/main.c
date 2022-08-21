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

/* USER CODE BEGIN Includes */
/* This code created for KETI Mobius platForm. */
/* 2016. 9.20. Lee So Haeng                    */
#include <stdio.h>
#include <string.h>
#include "tlcd.h"
#include "Base64.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t 	Rx_Data2[2];            // uart2 receive data buffer
uint8_t 	Rx_Data5[2];            // uart5 receive data buffer
uint8_t		Rx_Buffer[150];
                             //1234567890123456789012345678901
#define     TLCD_DATA	31
char 		KETI[TLCD_DATA] = "+* KETI Mobius platform Demo *+";
char 		KETIb[TLCD_DATA];

char 		DemoData[3][127] = { "at+data=8001ED:{\"id\":\"NODE1\",\"con\":\"123456789012345678901234567890123456789012345678901234567890123456789012345678901234567\"}\r\n",
							     "at+data=8001ED:{\"id\":\"NODE3\",\"con\":\"123456789012345678901234567890123456789012345678901234567890123456789012345678901234567\"}\r\n",
							     "at+data=8001ED:{\"id\":\"NODE4\",\"con\":\"123456789012345678901234567890123456789012345678901234567890123456789012345678901234567\"}\r\n"
						      };
/*
char 		DemoData[3][79] = {  "at+data=000194:{\"id\":\"NODE1\",\"con\":\"010203040506070809101qaz2wsx3edc4rfhy6y\"}\r\n",
							     "at+data=000194:{\"id\":\"NODE3\",\"con\":\"ABCDEFGHIJKLNMOPQRSTUVWXYZ!@#$%^&*()098\"}\r\n",
							     "at+data=000194:{\"id\":\"NODE4\",\"con\":\"12.34,34.23,05.55,90.88,87.90,1.234,897\"}\r\n"
						      }; */

// 디바이스의 시리얼번호를 확인할것...

volatile uint8_t     	TLCD_ON = 0;   //  Must volatile var!!
volatile uint8_t		TLCD_LS = 0;   //  Must volatile var!!
volatile uint8_t		Transfer2 = 0,Transfer5 = 0, DemoCnt = 0;
uint8_t					cnt,Rx_Data5_cnt = 0;
int 					len, elen;
unsigned char 			*e,*f,dt[105];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t buf_v, n;

	if(huart->Instance==USART2){
		HAL_UART_Transmit(&huart2,(uint8_t *)Rx_Data2, 1,1000000);
		HAL_UART_Receive_IT(&huart2,(uint8_t *)Rx_Data2, 1);
	}
	else if(huart->Instance==UART5){
		if(Rx_Data5_cnt==0) for(n=0;n<150;n++) Rx_Buffer[n] = 0x00;
		buf_v = Rx_Data5[0];
		if(buf_v!='\n' && buf_v!=0x00) {
			Rx_Buffer[Rx_Data5_cnt++] = buf_v;
		}
		else if(buf_v=='\n'){
			Rx_Buffer[Rx_Data5_cnt] = buf_v;
			Transfer5 = 1;
			Rx_Data5_cnt = 0;
		}
		HAL_UART_Receive_IT(&huart5,(uint8_t *)Rx_Data5, 1);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	uint8_t snum = 0;

	if(htim->Instance==TIM2) {       // 1 sec interrupt
		if(DemoCnt==3) DemoCnt = 0;
		//HAL_UART_Transmit(&huart5,(uint8_t *)&DemoData[DemoCnt],79,1000000);
		HAL_UART_Transmit(&huart5,(uint8_t *)&dt,105,1000000);
		DemoCnt++;
	}
	else if(htim->Instance==TIM3) {  // 0.5 sec interrupt
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		if(TLCD_LS==TLCD_DATA) TLCD_LS = 0;
		for(cnt=TLCD_LS;cnt<TLCD_DATA;cnt++) {
			KETIb[snum++] = KETI[cnt];
		}
		for(cnt=0;cnt<TLCD_LS;cnt++) {
			KETIb[snum++] = KETI[cnt];
		}
		TLCD_LS++;
		TLCD_ON = 1;
	}
	else if(htim->Instance==TIM4) {  // 0.1 sec interrupt
		  //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
}

/* USER CODE END 0 */

int main(void)
{
	unsigned char snd_str[65]= {0x01,0x01,0x02,0x00,};
  /* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  memset(dt,0x00,105);
  memcpy(dt,"at+data=0001A5:",15);
  e = base64_encode(snd_str,65,&elen);
  memcpy(dt+15,e+0,88);
  dt[103] = '\r';
  dt[104] = '\n';

 	//DemoData[0][77] = DemoData[1][77] = DemoData[2][77] = '\r';
 	//DemoData[0][78] = DemoData[1][78] = DemoData[2][78] = '\n';

 	// 1byte가 전송될때마다 인터럽트 발생.
	while(HAL_UART_Receive_IT(&huart2, (uint8_t *)Rx_Data2, 1)!=HAL_OK);
	while(HAL_UART_Receive_IT(&huart5, (uint8_t *)Rx_Data5, 1)!=HAL_OK);
	while(HAL_TIM_Base_Start_IT(&htim2)!=HAL_OK);
	while(HAL_TIM_Base_Start_IT(&htim3)!=HAL_OK);
	while(HAL_TIM_Base_Start_IT(&htim4)!=HAL_OK);

	// LCD사용을 위해 GPIOC를 사용. RS,E, D4,D5,D6,D7
	TLCD_Init(GPIOC,GPIO_PIN_10,GPIO_PIN_11,GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_2,GPIO_PIN_3);
	TLCD_Puts(1,0,"****************");
	TLCD_Puts(2,0,"* LoRA  Device *");

  /* USER CODE END 2 */
	HAL_UART_Transmit(&huart2,(uint8_t *)"This is \r\n",10,1000000);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  if(TLCD_ON) {
		  TLCD_Puts(1,0,KETIb);
		  TLCD_ON = 0;
	  }
	  HAL_UART_Transmit(&huart2,(uint8_t *)e,88,1000000);
	  HAL_Delay(1000000);
	  // uart5에서 데이터가 전송되면 가공후 usart2로 전송한다.
	  if(Transfer5 == 1) {
		  HAL_UART_Transmit(&huart2,(uint8_t *)Rx_Buffer,strlen((char *)Rx_Buffer),1000000);
		  Transfer5 = 0;
	  }

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 49999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 9000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* UART5 init function */
static void MX_UART5_Init(void)
{

  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 
                           PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

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
