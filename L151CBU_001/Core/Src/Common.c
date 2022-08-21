#include "Common.h"

uint8_t HEX_DECODE_BUFF[100];

void PrintInfo(void)
{
	TRACE_DEBUG("                                                 ");
	TRACE_DEBUG("=================================================");
	TRACE_DEBUG("         _____ __  ______    ____  ______        ");
	TRACE_DEBUG("        / ___//  |/  /   |  / __ \\/ ____/       ");
	TRACE_DEBUG("        \\__ \\/ /|_/ / /| | / /_/ / /_          ");
	TRACE_DEBUG("       ___/ / /  / / ___ |/ _, _/ __/            ");
	TRACE_DEBUG("      /____/_/  /_/_/  |_/_/ |_/_/               ");
	TRACE_DEBUG("                                                 ");
	TRACE_DEBUG("=================================================");
	TRACE_DEBUG("  Application : %s", APPLICATION_INFO);
	TRACE_DEBUG("  Build Date  : %s %s", __DATE__, __TIME__);
	TRACE_DEBUG("=================================================");

    return;
}

int32_t HexEncode(uint8_t *pInStr, uint8_t *pOutBuff, int32_t outBuffLen)
{
    uint8_t temp[3];
    int32_t i;
    int32_t outBufIdx = 0;
    memset(temp, 0x0, sizeof(temp));

    for(i = 0; i < strlen(pInStr); i+=2) {
        temp[0] = pInStr[i];
        temp[1] = pInStr[i+1];
        temp[3] = '\0';

        pOutBuff[outBufIdx] = strtol(temp, 0, 16);

        if(outBufIdx+1 > outBuffLen) {
            return outBufIdx;
        }
        else {
            outBufIdx++;
        }
    }
	
    return outBufIdx;
}

int32_t HexDecode(uint8_t *pInBuff, int32_t inBuffSize, uint8_t **ppOutStr)
{
    int32_t i, hexDecodeBuff;
    memset(HEX_DECODE_BUFF, 0x0, sizeof(HEX_DECODE_BUFF));

    if(inBuffSize > sizeof(HEX_DECODE_BUFF) / 2) {
        *ppOutStr = "";
        return -1;
    }

    for(i = 0, hexDecodeBuff = 0; i < inBuffSize; i++) {
        sprintf(&HEX_DECODE_BUFF[hexDecodeBuff], "%02x", pInBuff[i]);
        hexDecodeBuff += 2;
    }

    *ppOutStr = HEX_DECODE_BUFF;
    return 0;
}