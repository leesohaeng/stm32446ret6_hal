#ifndef __COMMON_H__
#define __COMMON_H__

#include "stm32l1xx_hal.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

// Application type define
#define APPLICATION_INFO "LOM202A Valve Application"


#define ANSI_COLOR_RED                      (0x031)
#define ANSI_COLOR_GREEN                    (0x032)
#define ANSI_COLOR_YELLOW                   (0x033)
#define ANSI_COLOR_BLUE                     (0x034)
#define ANSI_COLOR_MAGENTA                  (0x035)
#define ANSI_COLOT_CYAN                     (0x036)

#define TRACE_COLOR(COLOR_CODE, fmt, ...)   printf("\033[%xm"fmt"\033[m\n", COLOR_CODE, ##__VA_ARGS__)
#define TRACE_COLOR_RED(fmt, ...)           printf("\033[%xm"fmt"\033[m\n", ANSI_COLOR_RED, ##__VA_ARGS__)
#define TRACE_COLOR_GREEN(fmt, ...)         printf("\033[%xm"fmt"\033[m\n", ANSI_COLOR_GREEN, ##__VA_ARGS__)
#define TRACE_COLOR_YELLOW(fmt, ...) 		printf("\033[%xm"fmt"\033[m\n", ANSI_COLOR_YELLOW, ##__VA_ARGS__)
#define TRACE_COLOR_BLUE(fmt, ...) 			printf("\033[%xm"fmt"\033[m\n", ANSI_COLOR_BLUE, ##__VA_ARGS__)
#define TRACE_COLOR_MAGENTA(fmt, ...) 		printf("\033[%xm"fmt"\033[m\n", ANSI_COLOR_MAGENTA, ##__VA_ARGS__)
#define TRACE_COLOR_CYAN(fmt, ...) 			printf("\033[%xm"fmt"\033[m\n", ANSI_COLOT_CYAN, ##__VA_ARGS__)

#define TRACE_DEBUG_NOLF(fmt, ...)      	printf(fmt, ##__VA_ARGS__)
#define TRACE_DEBUG(fmt, ...)           	printf(fmt"\n", ##__VA_ARGS__)
#define TRACE_ARRAY_MSG_YELLOW(msg, array, length) 								\
		printf("\033[%xm"msg" [", ANSI_COLOR_YELLOW); 							\
		for(int i = 0; i < length; i++) 										\
			if((array[i] != '\n') && (array[i] != '\r')) printf("%c", array[i]);\
		printf("]\033[m\n");
#define TRACE_ARRAY_MSG_MAGENTA(msg, array, length) 							\
		printf("\033[%xm"msg" [", ANSI_COLOR_MAGENTA); 							\
		for(int i = 0; i < length; i++) 										\
			if((array[i] != '\n') && (array[i] != '\r')) printf("%c", array[i]);\
		printf("]\033[m\n");
#define TRACE_ARRAY_MSG_GREEN(msg, array, length) 							\
		printf("\033[%xm"msg" [", ANSI_COLOR_GREEN); 							\
		for(int i = 0; i < length; i++) 										\
			if((array[i] != '\n') && (array[i] != '\r')) printf("%c", array[i]);\
		printf("]\033[m\n");
#define TRACE_ARRAY_MSG(msg, array, length) 									\
		printf(msg" ["); 														\
		for(int i = 0; i < length; i++) 										\
			if((array[i] != '\n') && (array[i] != '\r')) printf("%c", array[i]);\
		printf("]\n");
#define SYSTEM_NOTICE(fmt, ...)         printf("\033[%xm""SYSTEM NOTI ["fmt"]\033[m\n", ANSI_COLOT_CYAN, ##__VA_ARGS__)

// Module status struct
typedef struct { 
	bool        LORA_JOIN_STATUS;
	bool 		ACK_SEND_TRIGGER;
	bool		VALVE_CONTROL_TRIGGER;
	bool        VALVE_STATUS;
	uint8_t     BATTERY_STATUS;
	uint32_t 	TOTAL_FLOW;
	uint8_t		DATA_SEND_PERIOD;
} COMMON_STATUS;


void PrintInfo(void);
int32_t HexEncode(uint8_t *pInStr, uint8_t *pOutBuff, int32_t outBuffLen);
int32_t HexDecode(uint8_t *pInBuff, int32_t inBuffSize, uint8_t **ppOutStr);
#endif