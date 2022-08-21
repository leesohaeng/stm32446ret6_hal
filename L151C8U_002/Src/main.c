/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l1xx_hal.h"
#include "Common.h"
#include "SystemClock.h"
#include "DebugSerial.h"
#include "BatteryAdcHandle.h"
#include "GpioHandle.h"
#include "LoraCommHandle.h"
#include "RtcHandle.h"

#define MODE_SELECT_TIMEOUT (5)
#define MODE_SELECT_KEY		'!'
#define MODE_SELECT_NORMAL 	0x00
#define MODE_SELECT_BYPASS 	0x01
/* Private variables ---------------------------------------------------------*/
COMMON_STATUS STATUS_COMMON;
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
int32_t ModeSelectGet(void) ;
/* Private function prototypes -----------------------------------------------*/

int main(void)
{
	int32_t selectMode;

	HAL_Init();
	SystemClock_Config();
	// Peripheral initialize
	TPS61291BypassDisable();
	LoraHandleGpio_Init();
	LoraCommSerial_Init();
	DebugSerial_Init();
	BatteryAdc_Init();
	ValveDrivePwrGpio_Init();
	HallSensorPwrGpio_Init();
	ValveHandleGpio_Init();
	FlowMeterGpio_Init();
	RTC_Init();
	
	// Print application infomation
	PrintInfo();
	// Mode select wait
	selectMode = ModeSelectGet();
	// Process start
	LoraModuleOn();

	while (1)
	{
		if (selectMode == MODE_SELECT_NORMAL) {
			SYSTEM_NOTICE("NORMAL MODE ENTER");
			RunNormalMode(&STATUS_COMMON);
		}
		else {
			SYSTEM_NOTICE("BYPASS MODE ENTER");
			LoraBypassMode();
		}
	}
}

int32_t ModeSelectGet(void) 
{
	int32_t timeoutCnt, modeSelect;
	uint8_t getCh;

	TRACE_DEBUG_NOLF("[PRESS '%c' KEY] : ENTER LORA BYPASS MODE ...", MODE_SELECT_KEY);

	timeoutCnt = 0;
	modeSelect = MODE_SELECT_NORMAL;

	while(timeoutCnt <= MODE_SELECT_TIMEOUT) {
		TRACE_DEBUG_NOLF("\b%d", (MODE_SELECT_TIMEOUT - timeoutCnt));
		if(DebugSerial_RxExist() > 0) {
			getCh = DebugSerial_GetByte();
			if(getCh == MODE_SELECT_KEY) {
				modeSelect = MODE_SELECT_BYPASS;
				break;
			}
		}
		timeoutCnt++;
		HAL_Delay(1000);
	}
	TRACE_DEBUG("");

	return modeSelect;
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char *file, int line)
{
	while (1)
	{
	}
	}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
		ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}

#endif

