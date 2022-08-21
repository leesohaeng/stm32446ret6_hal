#ifndef __BATTERY_ADC_HANDLE_H__
#define __BATTERY_ADC_HANDLE_H__

#include "stm32l1xx_hal.h"

void BatteryADC_Init(void);
uint8_t BatteryAdcGet(void);

#endif