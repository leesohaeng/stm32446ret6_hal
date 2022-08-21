
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "global.h"
#include "ssd1306.h"
#include "fonts.h"
//#include "stm32f1xx_hal_i2c.h"
#include "flash_eeprom.h"
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t RH, TEMP;
//uint8_t check = 0;
// int temp_low, temp_high, rh_low, rh_high;

//--------------------------
   uint8_t RX2_RingBuf[64];
   uint8_t RX2_Ptr=0;
   uint8_t RX2_Length=0;
   uint8_t RX2_rdPtr=0;
   uint8_t rx2_data=0;
   uint8_t rd2_data=0;
   uint8_t RX2_ComBuf[32];
   uint8_t rd2_Length=0;
   uint8_t RX2_ComWptr=0;
//--------------------------
   uint8_t RX1_RingBuf[64];
   uint8_t RX1_Ptr=0;
   uint8_t RX1_Length=0;
   uint8_t RX1_rdPtr=0;
   uint8_t rx1_data=0;
   uint8_t rd1_data=0;
   uint8_t RX1_ComBuf[32];
   uint8_t rd1_Length=0;
   uint8_t RX1_ComWptr=0;
//--------------------------
   uint8_t RX_ComBuf[32];
   uint8_t RX_ComWptr=0;
   uint8_t s_len;
   char * command_str[] = {"/s dt", "/s a"};
//--------------------------

   RTC_TimeTypeDef rtc_time;
   RTC_DateTypeDef rtc_date;
// RTC_AlarmTypeDef rtc_alarm;
   volatile uint16_t Buzer_Time=0;
   volatile uint16_t MsageTime=0;

   TIM_User_InitTypeDef tim;

   volatile bool oneSecond_flg = false;     // 1 초 마다 true
   volatile bool oneHundredms_flg = false;     // 100 ms 마다 true
   // volatile bool fiveHundredms_flg = false;    // 500 ms 마다 true
   volatile uint16_t led_delay_time = 1000;

   volatile uint16_t adcval[1];        // ADC1 ch1 번의 외부로 부터 입력받는 변수

/* Private variables ---------------------------------------------------------*/
   extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

   void CDC_TX_STR(char* str)
   {
	   CDC_Transmit_FS((uint8_t*)str, strlen(str));
   }

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//-------------------------------------
// 이부분의 정의는 USART2 로 printf 함수를 사용하기 위한 정의 입니다.

#ifdef __GNUC__
int _write(int file, uint8_t *ptr, int len)
{
	HAL_UART_Transmit(&huart2, ptr, len, 1000);
	return len;
}
#elif (__ICCARM__||__CC_ARM)
int fputc(int c, FILE* stream)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)&c,1, 1000);
	return c;
}
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  //==========================================
  // Flash Memory 초기화 및 User Data 저장
  // 한번 실행 하여 초기화 후 저장 했으면 주석 처리 해주세요..
      flash_memory_Init();
      flash_memory_erase();
      flash_memory_write();
  //==========================================

  // oled 128x64 초기화
    ssd1306_Init(&hi2c1);

    ssd1306_ClearScreen();  // ssd1306_Fill(Black);
    ssd1306_SetCursor(0,0);
    ssd1306_WriteString("OLED 128x64 Test", &Font_11x18, White);
    ssd1306_SetCursor(23, 18);
    ssd1306_WriteString("MyTest-7", &Font_11x18, White);
    ssd1306_UpdateScreen();

  //==================================
    HAL_UART_Receive_IT(&huart2,&rx2_data,1);	// 이렇게 1바이트 수신 함수를 호출 해야 수신 인터럽이 시작된다.
    HAL_UART_Receive_IT(&huart1,&rx1_data,1);	// 이렇게 1바이트 수신 함수를 호출 해야 수신 인터럽이 시작된다.
    HAL_TIM_Base_Start_IT(&htim2);			// 처음에 꼭 호출해줘야 TIM2 인터럽이 걸린다.
    HAL_TIM_Base_Start_IT(&htim4);         // 타이머4 인트럽트를 허용(ON)합니다


    printf("STM32F103C8 BluePill Board\r\5-3 OLED128x64_ssd1306-I2C2 Test!\r\n\r\n");		// 테스트 메세지.

    Buzer_Time = 100;

    char msg_str1[(SSD1306_WIDTH/8)+1];
    char msg_str2[(SSD1306_WIDTH/8)+1];
    char Temp_Rh_str[13];
    char str[35];

    char * weekday_str[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};

    uint32_t  count = 64;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    HAL_Delay(3000);
  //  time_start(&delay_time);
  while (1)
  {
	  if(oneSecond_flg)
	  {
		  HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);
		  HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
		  sprintf(msg_str1, "%04d-%02d-%02d %s",
				  rtc_date.Year+2000, rtc_date.Month, rtc_date.Date, weekday_str[rtc_date.WeekDay]);

		  sprintf(msg_str2, "%02d:%02d:%02d",
				  rtc_time.Hours, rtc_time.Minutes, rtc_time.Seconds);

		  ssd1306_ClearScreen();     // ssd1306_Fill(Black);
		  ssd1306_SetCursor(14,0);
		  ssd1306_WriteString(msg_str1, &Font_7x10, White);
		  ssd1306_SetCursor(19, 12);
		  ssd1306_WriteString(msg_str2, &Font_11x18, White);

		  ssd1306_UpdateScreen();

		///////////////메모리에서 데이터 읽기////////////////////////////////////////////////////////////
		// uart2 로 읽은 데이터를 전송한다.
		  printf("Addr = %X, Data = %d\r\n", USER_DATA1, *(__IO uint32_t *)USER_DATA1);
		  printf("Addr = %X, Data = %d\r\n", USER_DATA2, *(__IO uint32_t *)USER_DATA2);
		  printf("Addr = %X, Data = %d\r\n", USER_DATA3, *(__IO uint32_t *)USER_DATA3);
		  printf("Addr = %X, Data = %d\r\n", USER_DATA4, *(__IO uint32_t *)USER_DATA4);
		///////////////////////////////////////////////////////////////////////////////////////////////////

		  oneSecond_flg = false;
	  }
	  //if (oneHundredms_flg)
	  //{

	  //}


  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* RTC_Alarm_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
  /* USB_LP_CAN1_RX0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/* USER CODE BEGIN 4 */

//----------------------------------------------------
void sys_ini(void) // 초기화 함수입니다
{
}
//----------------------------------------------------
void chk_25ms(void) // per25msec
{
	if(led_delay_time == 25)
		   GPIOA->ODR ^= GPIO_PIN_15;
}
//----------------------------------------------------
void chk_100ms(void) // per100msec
{
	//oneHundredms_flg = true;
	if(led_delay_time == 100)
		   GPIOA->ODR ^= GPIO_PIN_15;
}
//----------------------------------------------------
void chk_500ms(void) // per500msec
{
	//fiveHundredms_flg = true;
	if(led_delay_time == 500)
	   GPIOA->ODR ^= GPIO_PIN_15; // PB12 토글 테스트를 합니다
   //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
}

//----------------------------------------------------
void chk_sec(void) // per1sec
{
	oneSecond_flg = true;
	if(led_delay_time == 1000)
		GPIOA->ODR ^= GPIO_PIN_15;
}
//----------------------------------------------------
void chk_tim(void) // 메인타이머 함수입니다
{
   //tim.flg = false; // 타이머플래그 끄기(OFF)를 합니다
   //----------------per25ms
   chk_25ms(); // 25msec 함수를 호출합니다
   //----------------
   if (++tim.tCnt < 4) return; // no 100ms?
   //----------------per100ms
   chk_100ms(); // 100msec 함수를 호출합니다
   //----------------
   tim.tCnt=0;
   if (++tim.t100ms < 5) return; // no 500ms?
   //----------------per500ms
   chk_500ms(); // 500msec 함수를 호출합니다
   //----------------
   tim.t100ms=0;
   if (++tim.t500ms & 0x1) return; // no 1sec?
   //----------------per1sec

   chk_sec(); // 1sec 함수를 호출합니다
   //----------------
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//===================================================
	// TIM2 = 1msec Interrupt
	if(htim->Instance==TIM2)
	{
		//----------------------------------------
		// HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);          // PC13 코글입니다. BluePill 보드
		//----------------------------------------
		if(MsageTime)MsageTime--;

		//----------------------------------------
		if(Buzer_Time)
		{
			Buzer_Time--;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
			if(Buzer_Time==0) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
		}
	}
	if(htim->Instance==TIM4)
	{
		chk_tim();
	}
}

//======================================================================================
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{	// 시리얼 수신 인터럽트 입니다.
	//======================================
	if(huart->Instance==USART1)
	{
		HAL_UART_Receive_IT(&huart1,&rx1_data,1);	// 1바이트 수신
		//HAL_UART_Transmit(&huart1,&rx1_data,1,10);	// 1바이트 송신(시리얼 수신데이터를 에코 백 하여 줍니다.)

		RX1_RingBuf[RX1_Ptr]=rx1_data;
		RX1_Length++;
		RX1_Ptr++;
		if(RX1_Ptr>63) RX1_Ptr=0;

		RX1_RingBuf[RX1_Ptr]=0;	// Null
	}
	//======================================
	if(huart->Instance==USART2)
	{
		HAL_UART_Receive_IT(&huart2,&rx2_data,1);	// 1바이트 수신
		//HAL_UART_Transmit(&huart2,&rx2_data,1,10);	// 1바이트 송신(시리얼 수신데이터를 에코 백 하여 줍니다.)

		uart2_read_command();
	}
}

void uart2_read_command(void)
{
	if((rx2_data == (uint8_t) '\n') || (rx2_data == (uint8_t) '\r'))
	{
		RX2_RingBuf[RX2_Ptr] = '\0';   // 마지막에 null 추가 하여 문자열의 끝을 알림

		if((RX2_Ptr >= 23) && (!strncasecmp((char *) RX2_RingBuf, command_str[0], 5)) )
		{
			set_Date_Time(&RX2_RingBuf[6]);    // Set Date Time 함수 호출
		}
		else if((RX2_Ptr >= 13) && (!strncasecmp((char *) RX2_RingBuf, command_str[1], 4)) )
		{
			set_Alarm_Time(&RX2_RingBuf[5]);   // Set Alarm Time 함수 호출
		}
		RX2_Ptr = 0;
	}
	else if(rx2_data == (uint8_t) '\b')  // Back Space
	{
		if(RX2_Ptr > 0)
			RX2_Ptr--;
	}
	else if(RX2_Ptr < 24)   // rxdata data Input to data_buf
	{
		RX2_RingBuf[RX2_Ptr++] = rx2_data;
	}
}

//=================================================================================================
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	printf("HAL RTC Alarm TEST OK!!\n\r");
}

// 외부 인터럽트 스위치 동작시 실행하는 함수
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
		case GPIO_PIN_15: led_delay_time = 25; break;
		case GPIO_PIN_14: led_delay_time = 100; break;
		case GPIO_PIN_13: led_delay_time = 500; break;
		case GPIO_PIN_12: led_delay_time = 1000;
	}
}

//===============================================================================

void set_Date_Time(uint8_t * dt_str)
{
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef DateToUpdate;

	for(uint8_t i=0; i<17; i++) dt_str[i] &= 0x0F;

	sTime.Hours = (dt_str[9]*10) + dt_str[10];
	sTime.Minutes = (dt_str[12]*10) + dt_str[13];
	sTime.Seconds = (dt_str[15]*10) + dt_str[16];
	while(HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK);

	uint8_t da = (dt_str[6]*10) + dt_str[7];
	uint8_t mo = (dt_str[3]*10) + dt_str[4];
	uint8_t ye = (dt_str[0]*10) + dt_str[1];
	DateToUpdate.WeekDay = day_of_week(ye, mo, da);
	DateToUpdate.Month = mo;
	DateToUpdate.Date  = da;
	DateToUpdate.Year  = ye;
	while(HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK);

}

void set_Alarm_Time(uint8_t * dt_str)
{
	RTC_AlarmTypeDef sAlarm;

	for(uint8_t i=0; i<8; i++) dt_str[i] &= 0x0F;

	sAlarm.AlarmTime.Hours = (dt_str[0]*10) + dt_str[1];      //0x17;
	sAlarm.AlarmTime.Minutes = (dt_str[3]*10) + dt_str[4];    //0x45;
	sAlarm.AlarmTime.Seconds = (dt_str[6]*10) + dt_str[7];    //0x0;
	sAlarm.Alarm = RTC_ALARM_A;
	while(HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
