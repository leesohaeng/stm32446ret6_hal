#ifndef __GPIO_HANDLE_H__
#define __GPIO_HANDLE_H__

#include "stm32l1xx_hal.h"
#include "Common.h"

void ValveDrivePwrGpio_Init(void);
void ValveDrivePwrControl(bool set);
void HallSensorPwrGpio_Init(void);
void HallSensorPwrControl(bool set);
void ValveHandleGpio_Init(void);
void ValveControl(bool set);
void FlowMeterGpio_Init(void);
uint32_t FlowMeterRead(void);

#endif