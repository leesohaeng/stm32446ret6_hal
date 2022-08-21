#include "RtcHandle.h"

/* Private define ------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef   RTC_HANDLE;
// RTC WAKEUP TRIGGER
bool WAKEUP_TRIGGER = false;
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

void RTC_Init(void)
{
    __HAL_RCC_RTC_ENABLE();

    /*Initialize RTC Only */
    RTC_HANDLE.Instance = RTC;
    RTC_HANDLE.Init.HourFormat = RTC_HOURFORMAT_24;
    RTC_HANDLE.Init.AsynchPrediv = 127;
    RTC_HANDLE.Init.SynchPrediv = 255;
    RTC_HANDLE.Init.OutPut = RTC_OUTPUT_DISABLE;
    RTC_HANDLE.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    RTC_HANDLE.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    if (HAL_RTC_Init(&RTC_HANDLE) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 0x03, 0);
    HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);

    return;
}

void RTC_WakeUpTimerSet(uint32_t waitSec)
{
    HAL_RTCEx_SetWakeUpTimer_IT(&RTC_HANDLE, waitSec, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);

    return;
}

void RTC_WakeUpTimerClear(void)
{
    HAL_RTCEx_DeactivateWakeUpTimer(&RTC_HANDLE);

    return;
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
	return;
}

bool RTC_WakeupTriggerGet(void)
{
	return WAKEUP_TRIGGER;
}

void RTC_WakeupTriggerClear(void)
{
	WAKEUP_TRIGGER = false;

    return;
}



void RTC_WKUP_IRQHandler(void)
{
    HAL_RTCEx_WakeUpTimerIRQHandler(&RTC_HANDLE);
    WAKEUP_TRIGGER = true;

    return;
}
