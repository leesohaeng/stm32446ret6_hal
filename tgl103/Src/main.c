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
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

#define a 	-6.08514057893257E+01
#define b 	1.15381410267267E-01
#define c 	-9.03230843887536E-05
#define d 	3.93417328130461E-08
#define e 	-5.50323888608191E-12
#define f 	-8.02107242737393E-16
#define g 	2.19258540076786E-19

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
                  //123456789012345678901234567890
char str[2][30] = {"Cortex-M3 Stm32f103c8t6.....\r\n",
		           "Mini device for simple proj.\r\n"};
uint8_t ui8 = 0, snum = 0;
char    mathStr[40];

volatile uint16_t con_ADCDATA[6];  // 239.5 clock data
volatile uint16_t gADCDATA[6];     // 1/10  sec data

/* Initialization of variables for kalman filter */
float x_next, x=0.0,P_next,P=1,K,Q = 1.0/100000.0,R = 0.1*0.1,z;

/*
name:	Polynomial Regression (degree=6)
kind:	Regression
family:	Linear Regressions
equation:	a + b*x + c*x^2 + ...
latexequation:	a + bx + cx^2 + ...
indep. variables:	1

Parameters:
#define a 	-6.08514057893257E+01
#define b 	1.15381410267267E-01
#define c 	-9.03230843887536E-05
#define d 	3.93417328130461E-08
#define e 	-5.50323888608191E-12
#define f 	-8.02107242737393E-16
#define g 	2.19258540076786E-19

Standard Error:	5.96915120304801E-01
Coefficient of Determination (r^2):	9.99859758426623E-01
Correlation Coefficient (r):	9.99929876754677E-01

Covariance matrix:
	6.7256551157488043E-01	-4.2665563936484761E-03	8.3301722971038438E-06	-7.1430399001585161E-09	3.0112763202003818E-12	-6.1327033615241933E-16	4.8239295117853225E-20
	-4.2665563940342344E-03	2.9821065406003806E-05	-6.1169363381753954E-08	5.4021532171007549E-11	-2.3222782172565127E-14	4.7955732416632437E-18	-3.8114653608579197E-22
	8.3301722988046165E-06	-6.1169363388213123E-08	1.3056948562071395E-10	-1.1857085634927897E-13	5.2002483531673306E-17	-1.0900628490726176E-20	8.7646131917726930E-25
	-7.1430399023752774E-09	5.4021532181845890E-11	-1.1857085635990790E-13	1.1000729526928750E-16	-4.9042549459578582E-20	1.0412139338440372E-23	-8.4575337835788817E-28
	3.0112763214166075E-12	-2.3222782179109826E-14	5.2002483540166041E-17	-4.9042549463060603E-20	2.2150281580602848E-23	-4.7521669575648385E-27	3.8932434772931442E-31
	-6.1327033644943946E-16	4.7955732433445482E-18	-1.0900628493169043E-20	1.0412139339772940E-23	-4.7521669578292982E-27	1.0283570752195092E-30	-8.4855116305669211E-35
	4.8239295144538814E-20	-3.8114653624172619E-22	8.7646131941817622E-25	-8.4575337850552054E-28	3.8932434776835284E-31	-8.4855116309396446E-35	7.0444008174390275E-39

Parameter Standard Deviations:
a_stddev =	4.89530636627185E-01
b_stddev =	3.25967391909424E-03
c_stddev =	6.82077033770437E-06
d_stddev =	6.26070619448553E-09
e_stddev =	2.80932643477402E-12
f_stddev =	6.05319340504220E-16
g_stddev =	5.00996404911345E-20

Parameter Uncertainties, 95%:
a_unc =	9.66595167959348E-01
b_unc =	6.43633885925543E-03
c_unc =	1.34678468657448E-05
d_unc =	1.23619808502641E-08
e_unc =	5.54711218031708E-12
f_unc =	1.19522396725767E-15
g_unc =	9.89234723875133E-20
*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	uint8_t t;
	double    fV,Resistance;
    double Temp = 0.0;
//    uint16_t Resistance;

	if(htim->Instance==TIM2) {
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
		for(t=0;t<2;t++) {
			gADCDATA[t] = con_ADCDATA[t];
		}

		// Assuming a 10k Thermistor.  Calculation is actually: Resistance = (4096/ADC)
		/******************************************************************/
		/* Utilizes the Steinhart-Hart Thermistor Equation:				*/
		/*    Temperature in Kelvin = 1 / {A + B[ln(R)] + C[ln(R)]^3}		*/
		/*    where A = 0.001129148, B = 0.000234125 and C = 8.76741E-08	*/
		/******************************************************************/

		Resistance=((40960000.0/(4095-(float)gADCDATA[0])) - 10000.0);
		fV = log(Resistance);
		Temp = 1 / (0.001129148 + (0.000234125 * fV));//  + (0.0000000876741 * fV * fV * fV));
		Temp = Temp - 298.15;  // Convert Kelvin to Celsius
		sprintf(mathStr,"[%6u%6.2f]",(gADCDATA[0]),Temp);
		HAL_UART_Transmit(&huart1, (uint8_t *)mathStr, 14,1000);
		// TLCD_Init(GPIOB,GPIO_PIN_3,GPIO_PIN_4,GPIO_PIN_5,GPIO_PIN_6,GPIO_PIN_7,GPIO_PIN_8);
		TLCD_Puts(1,0,mathStr);

		/*
		sprintf(mathStr,"*%10u%10.2f\r\n",gADCDATA[1],MMF(gADCDATA[1]));
		HAL_UART_Transmit(&huart1, (uint8_t *)mathStr, 23,1000);*/
		sprintf(mathStr,"[%10u]\r\n",gADCDATA[1]);
		TLCD_Puts(2,0,mathStr);
		HAL_UART_Transmit(&huart1, (uint8_t *)mathStr, 14,1000);
	}
	else if(htim->Instance==TIM3) {
/*		if(ui8==2) ui8 = 0;
		if(snum==100) snum = 0;
		sprintf(mathStr,"Sin(%03d) = %15.10lf\r\n",snum, sin(snum*3.141592/180));
		HAL_UART_Transmit(&huart1, (uint8_t *)mathStr, 28,1000);
		ui8++;
		snum++; */
	}
}

double MMF(uint16_t x)
{
	double r = (a + b*x + c*pow(x,2) + d*pow(x,3) + e*pow(x,4) + f*pow(x,5) +g*pow(x,6));
	return r;
}

float kalmanData(float input_data)
{
    float rtn;

    x_next=x;
    P_next=P+Q;                      //    1 + 0.00001
    K=P_next/(P_next+R);             //    1.00001 / (1.00001 + 0.01)
    z=input_data;                    // -0.37727 ** ; input data
    x=x_next+K*(z-x_next);           // 0 + 0.990099 *(input-0)
    P=(1-K)*P_next;

    rtn = x;
    return rtn;
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();

  TLCD_Init(GPIOB,GPIO_PIN_3,GPIO_PIN_4,GPIO_PIN_5,GPIO_PIN_6,GPIO_PIN_7,GPIO_PIN_8);
  TLCD_Puts(1,0,"12345678");
  TLCD_Puts(2,0,"asdfghjk");

  /* USER CODE BEGIN 2 */
  while(HAL_TIM_Base_Start_IT(&htim2)!=HAL_OK);
  while(HAL_TIM_Base_Start_IT(&htim3)!=HAL_OK);

  HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&con_ADCDATA,6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13, GPIO_PIN_SET);

  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 59999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1199;
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
  htim3.Init.Prescaler = 59999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 599;
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

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB3 PB4 PB5 
                           PB6 PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
