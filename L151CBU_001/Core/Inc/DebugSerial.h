#ifndef __DEBUG_SERIAL_H__
#define __DEBUG_SERIAL_H__

#include "stm32l1xx_hal.h"

void DebugSerial_Init(void);
void DebugSerial_RxFlush(void);
uint16_t DebugSerial_RxExist(void);
uint8_t DebugSerial_GetByte(void);
void DebugSerial_Send(uint8_t *pBuff, uint32_t buffLen);

#endif