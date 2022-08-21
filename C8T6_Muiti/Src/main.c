/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "my_mpu6050.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t Rx_Data2;            // uart2 receive data buffer
uint8_t Transfer_cplt2=0;
uint8_t count = 0,n;
char    sprt[50],gyroData[60],bufdata[14]={0,},i2cTxData[2],CaliBuf[7];
uint8_t MpuSetReady = 0,i2cStart=0;
volatile char Ten = 0,calibrate = 0;

/* Initialization of variables for kalman filter */
float alpha = 0.9,FSSEL=131;
float x_next[7], P_next[7], z[7],K[7],nn[7]={0,},nnadd[7]={0,},Caliave[7]={0,};
float P[7]={1,1,1,1,1,1,1};
float Q[7]={0.001,0.001,0.001,0.001,0.001,0.001,0.001};
float R[7]={0.02,0.02,0.02,0.02,0.02,0.02,0.02};
float xfilter_value[7]={0,0,0,0,0,0,0};

// gyro angle 계산용
float temp;
float gyro_x,gyro_y,gyro_z;
float accel_x,accel_y,accel_z;
float angle_x,angle_y,angle_z;
float accel_angle_x,accel_angle_y,accel_angle_z;
float gyro_angle_x,gyro_angle_y,gyro_angle_z;
float last_x_angle=0,last_y_angle=0,last_z_angle=0;
float last_gyro_x_angle=0,last_gyro_y_angle=0,last_gyro_z_angle=0;
float unfiltered_gyro_angle_x,unfiltered_gyro_angle_y,unfiltered_gyro_angle_z;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void kalman_filter();

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1){
		HAL_UART_Transmit(&huart1,(uint8_t *)&Rx_Data2, 1,1000);
		HAL_UART_Receive_IT(&huart1,(uint8_t *)&Rx_Data2, 1);
//		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
/*	if(huart->Instance==UART5){
		HAL_UART_Transmit(&huart2,(uint8_t *)&Rx_Data5, 1,1000);
		HAL_UART_Receive_IT(&huart5,(uint8_t *)&Rx_Data5, 1);
	} */
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	int n;

	if(htim->Instance==TIM2) {       // 1 sec interrupt
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}
	else if(htim->Instance==TIM3) {  // 1/100 sec interrupt
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		i2cTxData[0] = (uint8_t)0x3B;      // 0x3B - GYRO data register start address
		i2cTxData[1] = (uint8_t)0;         // 0
		while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)MPU6050,(uint8_t *)&i2cTxData[0],1,1000)!=HAL_OK);  // 0x3B
		while(HAL_I2C_Master_Receive (&hi2c1,(uint16_t)MPU6050,(uint8_t *)bufdata,14,1000)!=HAL_OK);

		for(n=0;n<7;n++)	nn[n] = ((int16_t)(bufdata[n*2]<<8)| bufdata[n*2+1]);
	    memset(bufdata,0x00,14);

		// 10ms에 합번씩 총 10회 읽어서 10으로 나누어 평균으로 구한 뒤 칼만필터로 계산한다.
		// 단, 첫 10회 데이터를 획득하여 평균을 구한다. 시작시 1번만......
		Ten++;
		if(Ten < 10) {
			for(n=0;n<7;n++) nnadd[n] += nn[n];
			return;
		}
		else Ten = 10;
		// 10번 읽어 평균을 구한다. 한번만 실행....
		if(!calibrate) {
			for(n=0;n<7;n++)	Caliave[n] = nnadd[n] / 10.0; // 초기 평균값을 저장한다.
			calibrate = 1;
		}

		kalman_filter();
	}

}

// 7개의 값을 칼만필터 적용.........................................................
void kalman_filter()
{
	int ifilter;
	char fstr[100];

	// **********************************
    // Iteration loop .. 칼만 필터 적용..
	// **********************************
    for(ifilter=0;ifilter<7;ifilter++) {
    	x_next[ifilter]=xfilter_value[ifilter];
    	P_next[ifilter]=P[ifilter]+Q[ifilter];
    	K[ifilter]=P_next[ifilter]/(P_next[ifilter]+R[ifilter]);
    	// Original Data : z
    	z[ifilter]=(float)nn[ifilter];
    	// Kalman   filter : xfilter_value
    	xfilter_value[ifilter]=x_next[ifilter]+K[ifilter]*(z[ifilter]-x_next[ifilter]);
    	P[ifilter]=(1-K[ifilter])*P_next[ifilter];
    }

    accel_x = xfilter_value[0];
    accel_y = xfilter_value[1];
    accel_z = xfilter_value[2];
    temp    =  (xfilter_value[3]+12412)/340.0;
    gyro_x = (xfilter_value[4] - Caliave[4]) / FSSEL;
    gyro_y = (xfilter_value[5] - Caliave[5]) / FSSEL;
    gyro_z = (xfilter_value[6] - Caliave[6]) / FSSEL;

    // accelator 값 보정. atan(y/sqrt(pow(x,2)+pow(y,2))
    accel_angle_x = atan(accel_y   /sqrt(pow(accel_x,2)+pow(accel_z,2)))*57.29579;
    accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2)+pow(accel_z,2)))*57.29579;
    accel_angle_z = 0;

    accel_angle_x = (int)(accel_angle_x*10)/10.0;
    accel_angle_y = (int)(accel_angle_y*10)/10.0;

    // gyro angle 계산 1
    gyro_angle_x = (int)((gyro_x*0.01 + last_x_angle)*10) / 10.0;
    gyro_angle_y = (int)((gyro_y*0.01 + last_y_angle)*10) / 10.0;
//    gyro_angle_z = gyro_z*0.01 + last_z_angle;
    gyro_angle_z = (int)((gyro_z*0.035)*10) / 10.0 + last_z_angle;

    // gyro angle 계산 2
    unfiltered_gyro_angle_x = (int)((gyro_x*0.01 + last_gyro_x_angle)*10) / 10.0;
    unfiltered_gyro_angle_y = (int)((gyro_y*0.01 + last_gyro_y_angle)*10) / 10.0;
    unfiltered_gyro_angle_z = (int)((gyro_z*0.035 + last_gyro_z_angle)*10) / 10.0;

    // 최종각도 계산
    angle_x = alpha * gyro_angle_x + (1.0 - alpha) * accel_angle_x;
    angle_y = alpha * gyro_angle_y + (1.0 - alpha) * accel_angle_y;
    angle_z = gyro_angle_z;

    // 마지막 각도 저장...
    last_x_angle = angle_x;
    last_y_angle = angle_y;
    last_z_angle = angle_z;
    last_gyro_x_angle = unfiltered_gyro_angle_x;
    last_gyro_y_angle = unfiltered_gyro_angle_y;
    last_gyro_z_angle = unfiltered_gyro_angle_z;

    sprintf(fstr,"ACC:%7.2f,%7.2f,%7.2f#GYR:%7.2f,%7.2f,%7.2f#FIL:%7.2f,%7.2f,%7.2f\r\n",
    		accel_angle_x,accel_angle_y,accel_angle_z,             	// ACCEL
			unfiltered_gyro_angle_x,unfiltered_gyro_angle_y,unfiltered_gyro_angle_z, // GYRO
			angle_x,angle_y,angle_z*-1		// Filtered
    );

    // 특성->c/c++build->setting->c linker->general->tool setting->runtime library->newlib standard
    HAL_UART_Transmit(&huart1,(uint8_t *)fstr, strlen(fstr),1000);
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  /*   pwm설정시 반드시 1ms로 설정할것.....
   *   htim2.Init.Prescaler = 71;
  	   htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  	   htim2.Init.Period = 999;
   *
   */
//  while(HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3 )!=HAL_OK);
//  while(HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4 )!=HAL_OK);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // START MPU6050 --------------------------------------------------------------------
  // 만약 mpu6050이 연결되어 있지 않으면 무한루프에 빠짐.

  i2cTxData[0] = (uint8_t)RA_PWR_ADDR;      // 0x6B ; power on
  i2cTxData[1] = (uint8_t) RA_PWR_MGMT_1;    // 0
  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)MPU6050,(uint8_t *)i2cTxData,2,1000)!=HAL_OK);
  HAL_Delay(200);    // wait 0.1 sec for MPU6050 START

  i2cTxData[0] = (uint8_t)RA_USER_CTRL;     // 0x6A ;
  i2cTxData[1] = (uint8_t)USER_CTRL;        // 0
  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)MPU6050,(uint8_t *)i2cTxData,2,1000)!=HAL_OK);

  i2cTxData[0] = (uint8_t)RA_INT_PIN_CFG;   // 0x37 ;
  i2cTxData[1] = (uint8_t)PIN_CFG;          // 0
  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)MPU6050,(uint8_t *)i2cTxData,2,1000)!=HAL_OK);

  i2cTxData[0] = (uint8_t)0x24;
  i2cTxData[1] = (uint8_t)20;
  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)MPU6050,(uint8_t *)i2cTxData,2,1000)!=HAL_OK);

  i2cTxData[0] = (uint8_t)RA_SMPLRT_DIV;    // 0x19 ;
  i2cTxData[1] = (uint8_t)SMPLRT_DIV;       // 19
  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)MPU6050,(uint8_t *)i2cTxData,2,1000)!=HAL_OK);

  i2cTxData[0] = (uint8_t)RA_GYRO_CONFIG;   // 0x1B ;
  i2cTxData[1] = (uint8_t)USER_CTRL;        // 0
  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)MPU6050,(uint8_t *)i2cTxData,2,1000)!=HAL_OK);

  i2cTxData[0] = (uint8_t)RA_ACCEL_CONFIG;  // 0x1C ;
  i2cTxData[1] = (uint8_t)USER_CTRL;        // 0
  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)MPU6050,(uint8_t *)i2cTxData,2,1000)!=HAL_OK);

  i2cTxData[0] = (uint8_t)RA_CONFIG;        // 0x1A ;
  i2cTxData[1] = (uint8_t)0;                // 0
  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)MPU6050,(uint8_t *)i2cTxData,2,1000)!=HAL_OK);

	// END   MPU6050 --------------------------------------------------------------------

  i2cTxData[0] = (uint8_t)RA_WHO_AM_I;      // 0x75 ; who am i
  i2cTxData[1] = (uint8_t)1;                // 0
  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)MPU6050,(uint8_t *)&i2cTxData[0],1,1000)!=HAL_OK);
  while(HAL_I2C_Master_Receive (&hi2c1,(uint16_t)MPU6050,(uint8_t *)whoAmI,2,1000)!=HAL_OK);

  while(HAL_TIM_Base_Start_IT(&htim1)!=HAL_OK);
  while(HAL_TIM_Base_Start_IT(&htim2)!=HAL_OK);
  while(HAL_TIM_Base_Start_IT(&htim3)!=HAL_OK);

//  int x = 0;
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  /*
	  for(x=0;x<=1000;x++) {
		  TIM2->CCR3 = x;
		  HAL_Delay(3);
	  }
	  HAL_Delay(700);
	  for(x=1000;x>=0;x--) {
		  TIM2->CCR3 = x;
		  HAL_Delay(3);
	  }
	  HAL_Delay(700); */
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

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig;
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Analog WatchDog 1 
    */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.HighThreshold = 0;
  AnalogWDGConfig.LowThreshold = 0;
  AnalogWDGConfig.Channel = ADC_CHANNEL_0;
  AnalogWDGConfig.ITMode = DISABLE;
  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
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

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7199;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4999;
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

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
