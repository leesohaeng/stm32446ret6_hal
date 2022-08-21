#ifndef __RTC_HANDLE_H__
#define __RTC_HANDLE_H__

#include "stm32l1xx_hal.h"
#include "Common.h"

#define RTC_WAKEUP_SEC (60)				// Rtc wakeup count (60sec)

void RTC_Init(void);
void RTC_WakeUpTimerSet(uint32_t waitSec);
void RTC_WakeUpTimerClear(void);
bool RTC_WakeupTriggerGet(void);
void RTC_WakeupTriggerClear(void);

#endif
