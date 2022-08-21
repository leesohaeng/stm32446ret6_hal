#include "BatteryAdcHandle.h"

/* Private define ------------------------------------------------------------*/
#define BATTERY_ADC_GPIO    GPIOA
#define BATTERY_ADC_PIN     GPIO_PIN_4
/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef 	HANDLE_BATTERY_ADC;
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

void BatteryAdc_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    ADC_ChannelConfTypeDef sConfig;

    // Battery adc input gpio clock enable
    __HAL_RCC_GPIOA_CLK_ENABLE();
    // Battery adc peripheral clock enable
    __HAL_RCC_ADC1_CLK_ENABLE();
    // Battery adc input gpio initialize
    GPIO_InitStruct.Pin     = BATTERY_ADC_PIN;
    GPIO_InitStruct.Mode    = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull    = GPIO_NOPULL;
    HAL_GPIO_Init(BATTERY_ADC_GPIO, &GPIO_InitStruct);
    // Battery adc peripheral initialize
    HANDLE_BATTERY_ADC.Instance = ADC1;
    HANDLE_BATTERY_ADC.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    HANDLE_BATTERY_ADC.Init.Resolution  = ADC_RESOLUTION_8B;
    HANDLE_BATTERY_ADC.Init.DataAlign   = ADC_DATAALIGN_RIGHT;
    HANDLE_BATTERY_ADC.Init.ScanConvMode = ADC_SCAN_DISABLE;
    HANDLE_BATTERY_ADC.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    HANDLE_BATTERY_ADC.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
    HANDLE_BATTERY_ADC.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
    HANDLE_BATTERY_ADC.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
    HANDLE_BATTERY_ADC.Init.ContinuousConvMode = DISABLE;
    HANDLE_BATTERY_ADC.Init.NbrOfConversion = 1;
    HANDLE_BATTERY_ADC.Init.DiscontinuousConvMode = DISABLE;
    HANDLE_BATTERY_ADC.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    HANDLE_BATTERY_ADC.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    HANDLE_BATTERY_ADC.Init.DMAContinuousRequests = DISABLE;
    if(HAL_ADC_Init(&HANDLE_BATTERY_ADC) != HAL_OK) {
        Error_Handler();
    }
    // Adc channel initialize
    sConfig.Channel = ADC_CHANNEL_4;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_16CYCLES;
    if(HAL_ADC_ConfigChannel(&HANDLE_BATTERY_ADC, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    return;
}

uint8_t BatteryAdcGet(void)
{
	uint32_t getAdcValue;
	
	if(HAL_ADC_Start(&HANDLE_BATTERY_ADC) != HAL_OK) {
		Error_Handler();
	}
	
	HAL_ADC_PollForConversion(&HANDLE_BATTERY_ADC, HAL_MAX_DELAY);

	if((HAL_ADC_GetState(&HANDLE_BATTERY_ADC) & HAL_ADC_STATE_EOC_REG) == HAL_ADC_STATE_EOC_REG) {
		getAdcValue = HAL_ADC_GetValue(&HANDLE_BATTERY_ADC);
	}

	if(HAL_ADC_Stop(&HANDLE_BATTERY_ADC) != HAL_OK) {
		Error_Handler();
	}

	return (uint8_t)getAdcValue;
}