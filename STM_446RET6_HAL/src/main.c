/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  * Date               : 2016.3.1
  * Programming by     : Lee So Haeng
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "main.h"
#include "tlcd.h"
#include "stm32f4xx_it.h"

/* Private variables ---------------------------------------------------------*/
#define DATA  10

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

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

uint8_t t = 0,tim2=0;
char s[20],s1[20];

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

static void MX_TIM2_Init(uint16_t Period);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void 		HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void 		HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void 		MX_ADC1_Init(void);
void 		MX_DMA_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
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
	  __GPIOC_CLK_ENABLE();
	  __GPIOH_CLK_ENABLE();
	  __GPIOA_CLK_ENABLE();
	  __GPIOB_CLK_ENABLE();

	  /*Configure GPIO pin : B1_Pin */
	  GPIO_InitStruct.Pin = B1_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : LD2_Pin */
	  GPIO_InitStruct.Pin = LD2_Pin | GPIO_PIN_8;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = GPIO_PIN_7;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pin Output Level */
	  // HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin | GPIO_PIN_8, GPIO_PIN_RESET);
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

/* TIM2 init function */
void MX_TIM2_Init(uint16_t Period)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 5999;
//  htim2.Init.Prescaler = 8;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = (15*Period)-1;     // 1 ms
//  htim2.Init.Period = (10*Period)-1;   // 1 us
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
}

/* TIM3 init function */
void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);

  HAL_TIM_MspPostInit(&htim3);
}

/* TIM4 init function */
void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 5999; //0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 14; // 0;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim4);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim4);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);

  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4);

  HAL_TIM_MspPostInit(&htim4);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM2) {
		if(t==91) t = 0;
		sprintf(s,"%16.10lf",arm_sin_f32(t*3.141592/180));
		t++;
		tim2 = 1;
	}
	else if(htim->Instance==TIM3) {

	}
	else if(htim->Instance==TIM4) {

	}
}

void MX_ADC1_Init(void)
{
	  ADC_ChannelConfTypeDef sConfig;

	    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	    */
	  hadc1.Instance = ADC1;
	  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
	  hadc1.Init.Resolution = ADC_RESOLUTION12b;
	  hadc1.Init.ScanConvMode = ENABLE;
	  hadc1.Init.ContinuousConvMode = ENABLE;
	  hadc1.Init.DiscontinuousConvMode = DISABLE;
	  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	  hadc1.Init.NbrOfConversion = 5;
	  hadc1.Init.DMAContinuousRequests = DISABLE;
	  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
	  HAL_ADC_Init(&hadc1);


    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 4;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

void Delay(__IO uint32_t nCount)
{
       /* Decrement nCountvalue */
	nCount *= 10;
       while (nCount != 0) {
    	   nCount--;
       }
}

/* USER CODE END 0 */

int main(void)
{
	/* USER CODE BEGIN 1 */
	uint32_t adcData[8];

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();

    /* Start ADC DMA */
//	MX_DMA_Init();
	MX_ADC1_Init();
	MX_DMA_Init();

    while(HAL_ADC_Start(&hadc1)!=HAL_OK);

	// Initialize Text LCD , GPIOB
	TLCD_Init(GPIO_PIN_4,GPIO_PIN_5,GPIO_PIN_12,GPIO_PIN_13,GPIO_PIN_14,GPIO_PIN_15);

	// Initialize USART1,2,3
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();

	// Initialize TIMER Interrupt2,3,4
	// TIM3,4 PWM
	MX_TIM2_Init(1000);
	MX_TIM3_Init();
	MX_TIM4_Init();

//	if(HAL_RCC_GetHCLKFreq()==180000000)	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,SET);

  /* USER CODE BEGIN 2 */
	// Activate USART RX1,RX2,RX3 interrupt every time receiving 1 byte
	while(HAL_UART_Receive_IT(&huart1, Rx_Data1, 1)!=HAL_OK);
	while(HAL_UART_Receive_IT(&huart2, Rx_Data2, 1)!=HAL_OK);
	while(HAL_UART_Receive_IT(&huart3, Rx_Data3, 1)!=HAL_OK);

	// Activate TIMER Interrupt
	while(HAL_TIM_Base_Start_IT(&htim2)!=HAL_OK);
	while(HAL_TIM_Base_Start_IT(&htim3)!=HAL_OK);
	while(HAL_TIM_Base_Start_IT(&htim4)!=HAL_OK);

	while(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcData, 8) != HAL_OK)  return 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
	   // If RX interrupt usart1,2,3.... and press Enter Key
		if(Transfer_cplt1)
		{
		//	HAL_UART_Transmit(&huart1, Rx_Buffer1, sizeof(Rx_Buffer1),1000);
		//	HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2,1000);
			Transfer_cplt1 = 0;
		}
		if(Transfer_cplt2)
		{
		//	HAL_UART_Transmit(&huart2, Rx_Buffer2, sizeof(Rx_Buffer2),1000);
		//	HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2,1000);
			Transfer_cplt2 = 0;
		}
		if(Transfer_cplt3)
		{
		//	HAL_UART_Transmit(&huart3, Rx_Buffer3, sizeof(Rx_Buffer3),1000);
		//	HAL_UART_Transmit(&huart3, (uint8_t *)"\r\n", 2,1000);
			Transfer_cplt3 = 0;
		}

		if(tim2==1)
		{
			TLCD_Puts(1,s);
		    sprintf(s1,"V  :  %10ld",adcData[0]);
			TLCD_Puts(2,s1);

			tim2 = 0;
		    sprintf(s1,"%10ld",adcData[0]);
			HAL_UART_Transmit(&huart2, (uint8_t *)s1,10 ,1000);
		    sprintf(s1,"%10ld",adcData[1]);
			HAL_UART_Transmit(&huart2, (uint8_t *)s1,10 ,1000);
		    sprintf(s1,"%10ld",adcData[2]);
			HAL_UART_Transmit(&huart2, (uint8_t *)s1,10 ,1000);
		    sprintf(s1,"%10ld",adcData[3]);
			HAL_UART_Transmit(&huart2, (uint8_t *)s1,10 ,1000);
		    sprintf(s1,"%10ld",adcData[4]);
			HAL_UART_Transmit(&huart2, (uint8_t *)s1,10 ,1000);
		    sprintf(s1,"%5ld",adcData[0]-adcData[4]);
			HAL_UART_Transmit(&huart2, (uint8_t *)s1,5 ,1000);
		    HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n",2 ,1000);
		    HAL_ADC_Stop_DMA(&hadc1);
		}
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		_delay_ms(100);

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
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
