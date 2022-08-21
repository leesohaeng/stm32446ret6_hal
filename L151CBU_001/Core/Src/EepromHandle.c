#include "EepRomHandle.h"

#define EEPROM_WRITE_FLAG_VALUE			0x33333333
#define EEPROM_USER_FLAG_ADDR			FLASH_EEPROM_BASE
#define EEPROM_USER_START_ADDR 			FLASH_EEPROM_BASE + 4


void EepromDataSave(COMMON_STATUS *pCommon)
{
	int i;
	uint32_t dataBuf;
	uint32_t eraseCnt, writeCnt;
	EEPROM_SAVE_STRUCT eepromSave;

	eepromSave.VALVE_STATUS 	= pCommon->VALVE_STATUS;
	
	eraseCnt = sizeof(EEPROM_SAVE_STRUCT) / sizeof(uint32_t);
	writeCnt = sizeof(EEPROM_SAVE_STRUCT) / sizeof(uint32_t);
	
	// EEPROM unlock
	HAL_FLASHEx_DATAEEPROM_Unlock();
	// EEPROM erase
	HAL_FLASHEx_DATAEEPROM_Erase(FLASH_TYPEERASEDATA_WORD, EEPROM_USER_FLAG_ADDR);
	for(i = 0; i < eraseCnt; i++) {
		HAL_FLASHEx_DATAEEPROM_Erase(FLASH_TYPEERASEDATA_WORD, EEPROM_USER_START_ADDR + (sizeof(uint32_t) * i));
	}
	// Write data
	for(i = 0; i < writeCnt; i++) {
		dataBuf = *(((uint32_t *)&eepromSave) + i);
		HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEERASEDATA_WORD, EEPROM_USER_START_ADDR + (sizeof(uint32_t) * i), dataBuf);
	}

	// WRITE FLAG VALUE
	HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEERASEDATA_WORD, EEPROM_USER_FLAG_ADDR, EEPROM_WRITE_FLAG_VALUE);
	HAL_FLASHEx_DATAEEPROM_Lock();
	
	return;
}


void EepromDataRead(COMMON_STATUS *pCommon)
{
	EEPROM_SAVE_STRUCT eepromSave;
	
	// EEPROM FLAG CHECK 
	if(*(uint32_t *)(EEPROM_USER_FLAG_ADDR) == EEPROM_WRITE_FLAG_VALUE) {
		memcpy((uint8_t *)&eepromSave, (uint8_t *)(EEPROM_USER_START_ADDR), sizeof(EEPROM_SAVE_STRUCT));

		pCommon->VALVE_STATUS 	= eepromSave.VALVE_STATUS;
	}
	
	return;
}
