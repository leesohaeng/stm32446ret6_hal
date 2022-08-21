/*
 * flash_eeprom.c
 *
 *  Created on: 2018. 11. 11.
 *      Author: TestParts
 */

#include "flash_eeprom.h"
// #include "stm32f1xx_hal_flash_ex.h"


// 데이터를 새로 쓰기위해서는 먼저 페이지 단위로 메모리를 지워 줘야 합니다.
static FLASH_EraseInitTypeDef EraseInitStruct;
static uint32_t PAGEError;

void flash_memory_Init(void)
{
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;  //0x00
	EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;  // 지우기 페이지의 시작 어드레스
	EraseInitStruct.NbPages     = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE; //지울 페이지 수
}

void flash_memory_erase(void)
{
	//Flash메모리를 조작 할 수 있도록 락을 풀어 줍니다.
	HAL_FLASH_Unlock();

	//페이지 지우기에 실패하면 무한루프에 빠지게 하여 기기의 오작동을 예방합니다.
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK) {
		printf("Eraser Error\r\n");
		while(1);
	}
}

void flash_memory_write(void)
{
	uint16_t test = 1000;

	//그리고 데이터 쓰기

	 /////////유저가 설정한 페이지에 데이터 쓰기 ////////////////////////////////////////////////////
	 //HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data)
	  while (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, USER_DATA1, ((uint32_t)test)) != HAL_OK){
		  //printf("Write Error\r\n");
	  }
	  while (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, USER_DATA2, ((uint32_t)0x00000000)) != HAL_OK){
		  //printf("Write Error\r\n");
	  }
	  while (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, USER_DATA3, ((uint32_t)0x00000001)) != HAL_OK){
		  //printf("Write Error\r\n");
	  }
	  while (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, USER_DATA4, ((uint32_t)0x00000002)) != HAL_OK){
		  //printf("Write Error\r\n");
	  }

	//마지막으로 Flash 메모리를 보호하기 위해 락을 걸어 줍니다.
	 HAL_FLASH_Lock();
}

void flash_memory_read(void)
{
	///////////////메모리에서 데이터 읽기////////////////////////////////////////////////////////////
	printf("Addr = %X, Data = %d\r\n", USER_DATA1, *(__IO uint32_t *)USER_DATA1);
	printf("Addr = %X, Data = %d\r\n", USER_DATA2, *(__IO uint32_t *)USER_DATA2);
	printf("Addr = %X, Data = %d\r\n", USER_DATA3, *(__IO uint32_t *)USER_DATA3);
	printf("Addr = %X, Data = %d\r\n", USER_DATA4, *(__IO uint32_t *)USER_DATA4);
	///////////////////////////////////////////////////////////////////////////////////////////////////
}


/*
 *
void flash_memory_write(void)
{
	uint16_t test = 1000;

	//그리고 데이터 쓰기

	 /////////유저가 설정한 페이지에 데이터 쓰기 ////////////////////////////////////////////////////
	 //HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data)
	  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, USER_DATA1, ((uint32_t)test)) != HAL_OK){
		  printf("Write Error\r\n");
		  while(1);
	  }
	  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, USER_DATA2, ((uint32_t)0x00000000)) != HAL_OK){
		  printf("Write Error\r\n");
		  while(1);
	  }
	  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, USER_DATA2, ((uint32_t)0x00000001)) != HAL_OK){
		  printf("Write Error\r\n");
		  while(1);
	  }
	  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, USER_DATA2, ((uint32_t)0x00000002)) != HAL_OK){
		  printf("Write Error\r\n");
		  while(1);
	  }

	//마지막으로 Flash 메모리를 보호하기 위해 락을 걸어 줍니다.
	 HAL_FLASH_Lock();
}
 */



