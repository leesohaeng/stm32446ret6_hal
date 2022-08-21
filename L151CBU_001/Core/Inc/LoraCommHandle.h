#ifndef __LORA_COMM_HANDLE_H__
#define __LORA_COMM_HANDLE_H__

#include "stm32l1xx_hal.h"
#include "Common.h"

#define MSG_JOIN_COMPLETE				"Join is completed"
#define MSG_ACK_RECEIVED                "Ack received"
#define MSG_RECV_BIN_MESSAGE            "Rx_MSG 1 : "

#define CLI_STX 						"LRW "
#define CLI_ETX							"\r\n"

#define CLI_SET_ACTIVATION_MODE			"30 "
#define CLI_TX_XXCONFIRM_MSG			"31 "
#define CLI_SET_REPORT_TIME				"32 "
#define CLI_SET_EUI						"33 "
#define CLI_GET_REPORT_TIME				"34 "
#define CLI_SET_DATARATE				"35 "
#define CLI_SET_ADR						"36 "
#define CLI_SET_RE_TX					"37 "
#define CLI_SET_LINK_CHECK				"38 "
#define CLI_SET_PROVISION_ONOFF			"3B "
#define CLI_SET_PROVISION_NONE_DONE		"3C "
#define CLI_GET_DEVICE_EUI				"3F "
#define CLI_GET_APP_EUI					"40 "
#define CLI_GET_TX_DATARATE				"42 "
#define CLI_GET_ADR						"44 "
#define CLI_GET_RE_TX					"45 "
#define CLI_GET_RX1_DELAY				"46 "
#define CLI_CHECK_SERIAL_CONNECTION		"49 "
#define CLI_GET_LAST_RSSI_SNR			"4A "
#define CLI_SET_CLASS_TYPE				"4B "
#define CLI_GET_CLASS_TYPE				"4C "
#define CLI_GET_FIRMWARE_VERSION		"4F "
#define CLI_GET_DEBUG_MSG_ONOFF			"50 "
#define CLI_SET_APPLICATION_KEY			"51 "
#define CLI_GET_APPLICATION_KEY			"52 "
#define CLI_SET_ATTEN					"53 "
#define CLI_GET_ATTEN					"63 "
#define CLI_SET_UNCONFIRM_MSG_RETRANS	"54 "
#define CLI_GET_UNCONFIRM_MSG_RETRANS	"55 "
#define CLI_GET_RX1_DATARATE_OFFSET		"56 "
#define CLI_SET_UPLINK					"58 "
#define CLI_GET_UPLINK					"59 "
#define CLI_SET_SLEEP_MODE				"60 "
#define CLI_SET_SOFT_RESET				"70 "
#define CLI_SET_CHANNEL_TX_POWER		"5E "
#define CLI_GET_CHANNEL_TX_POWER		"5F "
#define CLI_SET_TIME_SYNC_REQUEST		"39 "
#define CLI_TX_XXCONFIRM_BIN_DATA		"4D "

#define CMD_STATUS						0xA4
#define CMD_CONTROL						0xAA

#define DOWNLINK_CMD_VALVE_ON			0xFF
#define DOWNLINK_CMD_VALVE_OFF			0x00

#pragma pack(push,1) 
typedef struct {
	uint8_t 	COMMAND;
	uint8_t 	VALVE_STATUS;
	uint32_t	TOTAL_FLOW;
	uint8_t     BATTERY_STATUS;
	uint8_t 	DATA_SEND_PERIOD;
} MSG_FRAME_STATUS;

typedef struct {
	uint8_t 	COMMAND;
	uint8_t		VALVE_STATUS;
	uint8_t		DATA_SEND_PERIOD_SET;
	uint8_t		TOTAL_FLOW_RESET;
} MSG_FRAME_CONTROL;
#pragma pack(pop)


int32_t LoraMsgRecv(COMMON_STATUS *pCommon, uint32_t timeoutMs);
int32_t LoraFrameParse(uint8_t *pMsgFrame, uint32_t msgFrameLen, COMMON_STATUS *pCommon);
void LoraRxBinaryFrame(uint8_t *pData, uint32_t dataLen, COMMON_STATUS *pCommon);
void LoraTxBinaryFrame(uint8_t *pData, uint32_t dataLen, bool selectConfirmed);
void CommBinaryStatusSend(COMMON_STATUS *pCommon, bool selectConfirmed);

#endif