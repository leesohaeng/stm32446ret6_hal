#include "LoraCommHandle.h"
#include "Lom202aHandle.h"

/* Private define ------------------------------------------------------------*/
#define MAX_LORA_RX_BUFF    (128)
#define MAX_LORA_TX_BUFF    (128)
/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint8_t LORA_RX_BUF[MAX_LORA_RX_BUFF];
uint8_t LORA_TX_BUF[MAX_LORA_TX_BUFF];
uint8_t LORA_BIN_MSG_TEMP[MAX_LORA_TX_BUFF];
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

int32_t LoraMsgRecv(COMMON_STATUS *pCommon, uint32_t timeoutMs)
{
	int32_t err = -1;
	uint32_t tickStart, rxMsgBufLen;
	uint8_t getCh;

    // Clear Lora rx buff
	memset(LORA_RX_BUF, 0x00, sizeof(LORA_RX_BUF));
	// Start time tick set
	tickStart = HAL_GetTick();

    // Check time tick expired
  	while((HAL_GetTick() - tickStart) < timeoutMs)
	{
		if(LoraCommSerial_RxExist())
		{
			getCh = LoraCommSerial_GetByte();
				
			if((getCh == '\r') || (getCh == '\n')) {
				LORA_RX_BUF[rxMsgBufLen] = '\0';

                if(rxMsgBufLen > 0) 
                {
                    // Lora frame parse
                    err = LoraFrameParse(LORA_RX_BUF, rxMsgBufLen, pCommon);
				    rxMsgBufLen = 0;

					// Lora frame parse result
					if(err == 0) {
						break;
					}
                }
			}
			else {
				LORA_RX_BUF[rxMsgBufLen++] = getCh;
			}
			if(rxMsgBufLen >= MAX_LORA_RX_BUFF) {
				rxMsgBufLen = 0;
			}
		}
	}
	
	return err;
}

int32_t LoraFrameParse(uint8_t *pMsgFrame, uint32_t msgFrameLen, COMMON_STATUS *pCommon)
{
    uint8_t *pToken;
	int32_t loraBinMsgTempLen;

    // Frame type join complete
	if(strcmp(pMsgFrame, MSG_JOIN_COMPLETE) == 0) {
		TRACE_ARRAY_MSG_GREEN("LORA RECV", pMsgFrame, msgFrameLen);
        pCommon->LORA_JOIN_STATUS = true;
		return 0;
	}
    // Frame ack receive
    else if(strcmp(pMsgFrame, MSG_ACK_RECEIVED) == 0) {
		TRACE_ARRAY_MSG("LORA RECV", pMsgFrame, msgFrameLen);
		return 0;
	}
    // Frame rx binary message
    else if(strncmp(pMsgFrame, MSG_RECV_BIN_MESSAGE, strlen(MSG_RECV_BIN_MESSAGE)) == 0) {
		TRACE_ARRAY_MSG_MAGENTA("LORA RECV", pMsgFrame, msgFrameLen);

		pToken = &pMsgFrame[strlen(MSG_RECV_BIN_MESSAGE)];
        loraBinMsgTempLen = HexEncode(pToken, LORA_BIN_MSG_TEMP, sizeof(LORA_BIN_MSG_TEMP));
		LoraRxBinaryFrame(LORA_BIN_MSG_TEMP, loraBinMsgTempLen, pCommon);
		return -1; // MSG_ACK_RECEIVED 수신을 위하여 -1 Return
	}
	else {
		TRACE_ARRAY_MSG("LORA RECV", pMsgFrame, msgFrameLen);
		return -1;
	}

}

void LoraRxBinaryFrame(uint8_t *pData, uint32_t dataLen, COMMON_STATUS *pCommon)
{
	MSG_FRAME_CONTROL *pFrame;
	uint8_t *pTraceMsg;

	HexDecode(pData, dataLen, &pTraceMsg);
	SYSTEM_NOTICE("RECV BIN MSG : %s", pTraceMsg);

	// Command check
	if(pData[0] == CMD_CONTROL) {
		pFrame = (MSG_FRAME_CONTROL *)pData;
		// VALVE ON/OFF CHECK
		if(pFrame->VALVE_STATUS == DOWNLINK_CMD_VALVE_ON) 
		{
			if(pCommon->VALVE_STATUS != true) {
				SYSTEM_NOTICE("=========== SET VALVE ON ===========");
				pCommon->VALVE_STATUS = true;
				pCommon->VALVE_CONTROL_TRIGGER = true;
			}
			else {
				SYSTEM_NOTICE("===== EQUAL VALVE STATE( ON ) ======");
			}
		}
		else {
			if(pCommon->VALVE_STATUS != false) {
				SYSTEM_NOTICE("=========== SET VALVE OFF ==========");
				pCommon->VALVE_STATUS = false;
				pCommon->VALVE_CONTROL_TRIGGER = true;
			}
			else {
				SYSTEM_NOTICE("==== EQUAL VALVE STATE( OFF ) ======");
			}			
		}
		// DATA SEND PERIOD SET
		if((pFrame->DATA_SEND_PERIOD_SET > 0) && (pFrame->DATA_SEND_PERIOD_SET <= 10)) {
			pCommon->DATA_SEND_PERIOD = pFrame->DATA_SEND_PERIOD_SET;
			SYSTEM_NOTICE("===== DATA SEND PERIOD(%d Min) =====", pCommon->DATA_SEND_PERIOD);
		}
		else {
			pCommon->DATA_SEND_PERIOD = 1;
			SYSTEM_NOTICE("===== DATA SEND PERIOD(Default) ====");
		}
		
		// TOTAL FLOW RESET
		if(pFrame->TOTAL_FLOW_RESET == 1) {
			pCommon->TOTAL_FLOW = 0;
			SYSTEM_NOTICE("========= TOTAL FLOW RESET =========");
		}
		// ACK 데이터를 SEND 하기 위한 Trigger 를 TRUE로 SET
		pCommon->ACK_SEND_TRIGGER = true;
	}
}

void LoraTxBinaryFrame(uint8_t *pData, uint32_t dataLen, bool selectConfirmed)
{
	uint8_t *pTraceStr;
	uint32_t txMessageBufIdx = 0;
	memset(LORA_TX_BUF, 0x00, sizeof(LORA_TX_BUF));

	//STX
	memcpy(&LORA_TX_BUF[txMessageBufIdx], CLI_STX, (sizeof(CLI_STX) - 1));
	txMessageBufIdx += strlen(CLI_STX);
	// CID
	memcpy(&LORA_TX_BUF[txMessageBufIdx], CLI_TX_XXCONFIRM_BIN_DATA, (sizeof(CLI_TX_XXCONFIRM_BIN_DATA) - 1));
	txMessageBufIdx += strlen(CLI_TX_XXCONFIRM_BIN_DATA);
	// MTYPE
	if(selectConfirmed) {	
		LORA_TX_BUF[txMessageBufIdx] = 0x01;			
	}
	else {
		LORA_TX_BUF[txMessageBufIdx] = 0x00;
	}
  	txMessageBufIdx++;
	// FPORT
	LORA_TX_BUF[txMessageBufIdx] = 0x01;
	txMessageBufIdx++;
	// LENGTH
	LORA_TX_BUF[txMessageBufIdx] = dataLen;
	txMessageBufIdx++;
	// MESSAGE
	memcpy(&LORA_TX_BUF[txMessageBufIdx], pData, dataLen);
	txMessageBufIdx += dataLen;
	// ETX
	memcpy(&LORA_TX_BUF[txMessageBufIdx], CLI_ETX, (sizeof(CLI_ETX) - 1));
	txMessageBufIdx += strlen(CLI_ETX);

	// TRACE MESSAGE
	HexDecode(pData, dataLen, &pTraceStr);
	SYSTEM_NOTICE("SEND BIN MSG : %s", pTraceStr);
	TRACE_ARRAY_MSG_YELLOW("LORA SEND", LORA_TX_BUF, txMessageBufIdx);
	// SEND MESSAGE
	LoraCommSerial_Send(LORA_TX_BUF, txMessageBufIdx);

	return;
}

uint32_t StatusMsgMake(uint8_t *pData, COMMON_STATUS *pCommon)
{
	MSG_FRAME_STATUS *pMessage = (MSG_FRAME_STATUS *)pData;

	// Command write
	pMessage->COMMAND = CMD_STATUS;
	// Valve status set
	if(pCommon->VALVE_STATUS) {
		pMessage->VALVE_STATUS = DOWNLINK_CMD_VALVE_ON;
	}
	else {
		pMessage->VALVE_STATUS = DOWNLINK_CMD_VALVE_OFF;
	}
	// Total flow set
	pMessage->TOTAL_FLOW = pCommon->TOTAL_FLOW;
	// Battery status set
	pMessage->BATTERY_STATUS = pCommon->BATTERY_STATUS;
	// Data Send Period
	pMessage->DATA_SEND_PERIOD = pCommon->DATA_SEND_PERIOD;

	return sizeof(MSG_FRAME_STATUS);	
}


void CommBinaryStatusSend(COMMON_STATUS *pCommon, bool selectConfirmed)
{
	uint32_t dataLen;

	// Make status message
	dataLen = StatusMsgMake(LORA_BIN_MSG_TEMP, pCommon);
	// Make lora tx binary message and send frame
	LoraTxBinaryFrame(LORA_BIN_MSG_TEMP, dataLen, selectConfirmed);

	return;
}