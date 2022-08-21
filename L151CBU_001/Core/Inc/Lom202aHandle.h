#ifndef __LOM202A_HANDLE_H__
#define __LOM202A_HANDLE_H__

#include "stm32l1xx_hal.h"

void LoraHandleGpio_Init(void);
void LoraModuleOff(void);
void LoraModuleOn(void);
void LoraReset(void);
void LoraWakeUp(void);

void LoraCommSerial_Init(void);
void LoraCommSerial_RxFlush(void);
uint16_t LoraCommSerial_RxExist(void);
uint8_t LoraCommSerial_GetByte(void);
void LoraCommSerial_Send(uint8_t *pBuff, uint32_t buffLen);

#endif