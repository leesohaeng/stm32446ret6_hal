/*
 * flash_eeprom.h
 *
 *  Created on: 2018. 11. 11.
 *      Author: TestParts
 */

#ifndef FLASH_EEPROM_H_
#define FLASH_EEPROM_H_

#include "stm32f1xx_hal.h"
//#include "stm32f1xx_hal_flash.h"

// low(LD),medium(MD) density는 블럭이 1KB, High(HD) density는 2KB 단위임.
// STM32F1xx의 메모리 섹터입니다.

#define ADDR_FLASH_PAGE_63     ((uint32_t)0x0800FC00) /* Base @ of Page 63,  1 Kbyte */

#define ADDR_FLASH_PAGE_64     ((uint32_t)0x08010000) /* Base @ of Page 64,  1 Kbyte */
#define ADDR_FLASH_PAGE_65     ((uint32_t)0x08010400) /* Base @ of Page 65,  1 Kbyte */

#define ADDR_FLASH_PAGE_127    ((uint32_t)0x0801FC00) /* Base @ of Page 127,  1 Kbyte */


// 사용 자 시작 주소를 변경하 면 됩니다..
// ADDR_FLASH_PAGE_64,    ADDR_FLASH_PAGE_127
#define FLASH_USER_START_ADDR   ADDR_FLASH_PAGE_127  /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_PAGE_127 + FLASH_PAGE_SIZE   /* End @ of user Flash area */


// 그리고 Flash memory에 저장할 사용자 데이터를 정의 합니다.  예제에서는 4byte 변수를 사용하기 위해 어드레스를 4byte단위로 증가시켰습니다.

#define USER_DATA1     (FLASH_USER_START_ADDR)
#define USER_DATA2     (FLASH_USER_START_ADDR + 4)
#define USER_DATA3     (FLASH_USER_START_ADDR + 8)
#define USER_DATA4     (FLASH_USER_START_ADDR + 12)

// 페이지 단위로 지울수 있도록 구조체변수를 선언해 주고 멤버변수값들을 정해줍니다.

void flash_memory_Init(void);
void flash_memory_erase(void);
void flash_memory_write(void);
void flash_memory_read(void);


#endif /* FLASH_EEPROM_H_ */

