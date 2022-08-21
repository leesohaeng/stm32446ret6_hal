#ifndef __NORMAL_MODE_RUN_H__
#define __NORMAL_MODE_RUN_H__

#include "stm32l1xx_hal.h"
#include "Common.h"

void RunNormalMode(COMMON_STATUS *pCommon);
void CommonStatusDefaultSet(COMMON_STATUS *pCommon);
void Sequence_WaitJoin(COMMON_STATUS *pCommon);
void Sequence_GetSensorData(COMMON_STATUS *pCommon);
void Sequence_SendRecvData(COMMON_STATUS *pCommon);
void Sequence_SendAckData(COMMON_STATUS *pCommon);
void Sequence_ControlDevice(COMMON_STATUS *pCommon);
void Sequence_Sleep(COMMON_STATUS *pCommon);
#endif