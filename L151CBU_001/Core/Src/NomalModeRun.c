#include "NormalModeRun.h"
#include "BatteryAdcHandle.h"
#include "GpioHandle.h"
#include "RtcHandle.h"
#include "EepromHandle.h"

#define LORA_JOIN_WAIT_MS   	(60 * 1000)
#define LORA_CMD_MSG_WAIT_SEC	(60 * 1000)	// 서버의 COMMAND 메시지 TIMEOUT, 통신 환경이 좋지 않을 경우
											// 메시지를 수신 하는데 걸리는 시간이 증가하는 것을 확인함.

void RunNormalMode(COMMON_STATUS *pCommon)
{
	CommonStatusDefaultSet(pCommon);
	Sequence_WaitJoin(pCommon);

	while(1)
	{
		Sequence_GetSensorData(pCommon);
		Sequence_SendRecvData(pCommon);
		Sequence_ControlDevice(pCommon);
		Sequence_SendAckData(pCommon);
		Sequence_Sleep(pCommon);
		//HAL_Delay(10 * 1000);
	}
}

void CommonStatusDefaultSet(COMMON_STATUS *pCommon)
{
	// Common status value default set 
	pCommon->LORA_JOIN_STATUS = false;
	pCommon->VALVE_CONTROL_TRIGGER = false;
	pCommon->ACK_SEND_TRIGGER = false;
	pCommon->VALVE_STATUS = false;
	pCommon->TOTAL_FLOW = 0;
	pCommon->BATTERY_STATUS = 0;
	pCommon->DATA_SEND_PERIOD = 1;
	// Load eeprom
	EepromDataRead(pCommon);
	// Restore before valve status
	if(pCommon->VALVE_STATUS) {
		SYSTEM_NOTICE("==== RESTORE VALVE STATE( ON ) ======");
		ValveControl(pCommon->VALVE_STATUS);
		// Hall sensor power enable
		HallSensorPwrControl(true);
	}
	else {
		SYSTEM_NOTICE("==== RESTORE VALVE STATE( OFF ) =====");
		ValveControl(pCommon->VALVE_STATUS);
		// Hall sensor power disable
		HallSensorPwrControl(false);
	}
}

void Sequence_WaitJoin(COMMON_STATUS *pCommon)
{	
    LoraMsgRecv(pCommon, LORA_JOIN_WAIT_MS);
    // Check join result
    if(!pCommon->LORA_JOIN_STATUS) {
        SYSTEM_NOTICE("Join fail... system reset");
        // Join fail system reset
        NVIC_SystemReset();
    }
    SYSTEM_NOTICE("Join complete... sequence run");
	return;
}

void Sequence_GetSensorData(COMMON_STATUS *pCommon)
{
	int32_t 	err;
	uint8_t 	batteryAdc;
	
	// Get battery ADC value
	batteryAdc = BatteryAdcGet();
	SYSTEM_NOTICE("Battery ADC : %d", batteryAdc);
    pCommon->BATTERY_STATUS = batteryAdc;
	pCommon->TOTAL_FLOW += FlowMeterRead();
	SYSTEM_NOTICE("Total Flow : %d", pCommon->TOTAL_FLOW);

	return;
}

void Sequence_SendRecvData(COMMON_STATUS *pCommon)
{
	// CONFIRMED UP 
	CommBinaryStatusSend(pCommon, true);
	LoraMsgRecv(pCommon, LORA_CMD_MSG_WAIT_SEC);
	
	return;
}

void Sequence_SendAckData(COMMON_STATUS *pCommon)
{
	if(pCommon->ACK_SEND_TRIGGER) {
		// UNCONFIRMED UP
		CommBinaryStatusSend(pCommon, false);
		pCommon->ACK_SEND_TRIGGER = false;
	}
	
	return;
}

void Sequence_ControlDevice(COMMON_STATUS *pCommon)
{
	// Valve control
	if(pCommon->VALVE_CONTROL_TRIGGER) {
		SYSTEM_NOTICE("VALVE STATE REFRESH");
		// Valve control
		ValveControl(pCommon->VALVE_STATUS);
		// Hall sensor control
		if(pCommon->VALVE_STATUS) {
			// Hall sensor power enable
			HallSensorPwrControl(true);
		}
		else {
			// Hall sensor power disable
			HallSensorPwrControl(false);
		}
		EepromDataSave(pCommon);
		// Clear valve control trigger
		pCommon->VALVE_CONTROL_TRIGGER = false;
	}
	
}

#if 0
void Sequence_Sleep(void)
{
	SYSTEM_NOTICE("ENTER SLEEP MODE");
	// RTC INTERRUPT ENABLE
	RTC_WakeUpTimerSet(RTC_WAKEUP_SEC);
	// STOP TICK COUNT INTERRUPT
	HAL_SuspendTick();
	// ENTER STOP MODE
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	// CLOCK RECONFIGURE
	SystemClock_Config();
	// RESUME TICK COUNT INTERRUPT
	HAL_ResumeTick();
	// RTC INTERRUPT DISABLE	
	RTC_WakeUpTimerClear();
	SYSTEM_NOTICE("EXIT SLEEP");
}
#endif

void Sequence_Sleep(COMMON_STATUS *pCommon)
{
	SYSTEM_NOTICE("ENTER SLEEP MODE");
	// RTC WAKEUP TRIGGER RESET
	RTC_WakeupTriggerClear();
	// RTC INTERRUPT ENABLE
	RTC_WakeUpTimerSet(pCommon->DATA_SEND_PERIOD * 60);
	
	// RTC INTERRUPT 가 발생하여 Wakeup trigger가 set 될 때 까지 슬립모드 재진입
	while(!RTC_WakeupTriggerGet())
	{
		// STOP TICK COUNT INTERRUPT
		HAL_SuspendTick();
		// ENTER STOP MODE
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
		// Wakeup 또는 GPIO Interrupt 가 발생 한 경우 Wakeup 한다
		// Wakeup 지점 ////////////////////////////////////////////////////
		// CLOCK RECONFIGURE
		SystemClock_Config();
		// RESUME TICK COUNT INTERRUPT
		HAL_ResumeTick();
	}
	
	// RTC INTERRUPT DISABLE	
	RTC_WakeUpTimerClear();
	SYSTEM_NOTICE("EXIT SLEEP");
}