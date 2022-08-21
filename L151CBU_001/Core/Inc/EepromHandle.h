#ifndef __EEPROM_HANDLE_H__
#define __EEPROM_HANDLE_H__

#include "stm32l1xx_hal.h"
#include "Common.h"

#pragma pack(push,4) 
typedef struct {
	uint32_t 		VALVE_STATUS;
} EEPROM_SAVE_STRUCT;
#pragma pack(pop)

void EepromDataSave(COMMON_STATUS *pCommon);
void EepromDataRead(COMMON_STATUS *pCommon);

#endif