#include "GpioHandle.h"

/* Private define ------------------------------------------------------------*/
// Valve handle gpio define
#define VALVE_HANDLE_GPIO   	GPIOB
#define VALVE_POWER_PIN     	GPIO_PIN_3
#define VALVE_ON_PIN        	GPIO_PIN_5
#define VALVE_OFF_PIN       	GPIO_PIN_4

#define VALVE_DRIVE_PWR_GPIO 	GPIOB
#define VALVE_DRIVE_PWR_PIN 	GPIO_PIN_6

#define HALL_SENSOR_PWR_GPIO 	GPIOB
#define HALL_SENSOR_PWR_PIN		GPIO_PIN_7

#define HAL_SENSOR_INPUT_GPIO 	GPIOA
#define HAL_SENSOR_INPUT_PIN 	GPIO_PIN_6
/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint32_t FLOW_METER_COUNT;
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

void ValveDrivePwrGpio_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	// Valve drive power handle gpio clock enable
	__HAL_RCC_GPIOB_CLK_ENABLE();
	// Valve drive power handle gpio initialize
	GPIO_InitStruct.Pin = VALVE_DRIVE_PWR_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(VALVE_DRIVE_PWR_GPIO, &GPIO_InitStruct);

	// Default value set
	ValveDrivePwrControl(false);
	return;
}

void ValveDrivePwrControl(bool set)
{
	if(set) {
		HAL_GPIO_WritePin(VALVE_DRIVE_PWR_GPIO, VALVE_DRIVE_PWR_PIN, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(VALVE_DRIVE_PWR_GPIO, VALVE_DRIVE_PWR_PIN, GPIO_PIN_RESET);	
	}

	return;
}

void HallSensorPwrGpio_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	// Hall sensor power handle gpio clock enable
	__HAL_RCC_GPIOB_CLK_ENABLE();
	// Hall sensor power handle gpio initialize
	GPIO_InitStruct.Pin = HALL_SENSOR_PWR_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(HALL_SENSOR_PWR_GPIO, &GPIO_InitStruct);

	// Default value set
	HallSensorPwrControl(false);
	return;
}

void HallSensorPwrControl(bool set)
{
	if(set) {
		HAL_GPIO_WritePin(HALL_SENSOR_PWR_GPIO, HALL_SENSOR_PWR_PIN, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(HALL_SENSOR_PWR_GPIO, HALL_SENSOR_PWR_PIN, GPIO_PIN_RESET);	
	}

	return;
}

void ValveHandleGpio_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
    // Valve handle gpio clock enable
    __HAL_RCC_GPIOB_CLK_ENABLE();
	// Valve gpio initialize
	GPIO_InitStruct.Pin = 	VALVE_POWER_PIN;
	GPIO_InitStruct.Mode = 	GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = 	GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(VALVE_HANDLE_GPIO	, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = 	VALVE_ON_PIN;
	GPIO_InitStruct.Mode = 	GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = 	GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(VALVE_HANDLE_GPIO	, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = 	VALVE_OFF_PIN;
	GPIO_InitStruct.Mode = 	GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = 	GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(VALVE_HANDLE_GPIO	, &GPIO_InitStruct);

	ValveControl(false);

    return;
}

void ValveControl(bool set)
{
	ValveDrivePwrControl(true);

	// Valve driver power on
	HAL_GPIO_WritePin(VALVE_HANDLE_GPIO, VALVE_POWER_PIN, GPIO_PIN_SET);
	HAL_Delay(100);
		
	if(set) {
		// Valve on
		HAL_GPIO_WritePin(VALVE_HANDLE_GPIO, VALVE_ON_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(VALVE_HANDLE_GPIO, VALVE_OFF_PIN, GPIO_PIN_RESET);
	}
	else {
		// Valve off
		HAL_GPIO_WritePin(VALVE_HANDLE_GPIO, VALVE_ON_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(VALVE_HANDLE_GPIO, VALVE_OFF_PIN, GPIO_PIN_SET);
	}

	HAL_Delay(600);
  	HAL_GPIO_WritePin(VALVE_HANDLE_GPIO, VALVE_ON_PIN, GPIO_PIN_RESET);
  	HAL_GPIO_WritePin(VALVE_HANDLE_GPIO, VALVE_OFF_PIN, GPIO_PIN_RESET); 

	HAL_Delay(100);
	HAL_GPIO_WritePin(VALVE_HANDLE_GPIO, VALVE_POWER_PIN, GPIO_PIN_RESET);

	ValveDrivePwrControl(false);
	
	return;
}

void FlowMeterGpio_Init(void)
{
	static GPIO_InitTypeDef  GPIO_InitStruct;
	// Hall sensor input gpio clock enable
	__HAL_RCC_GPIOA_CLK_ENABLE();
	// Hall sensor input gpio configuration
	GPIO_InitStruct.Pin = 	HAL_SENSOR_INPUT_PIN;
	GPIO_InitStruct.Mode = 	GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(HAL_SENSOR_INPUT_GPIO	, &GPIO_InitStruct);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 3);
  	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	// Clear flow meter count
	FLOW_METER_COUNT = 0;

	return;
}

uint32_t FlowMeterRead(void)
{
	float tempF;

	// Calculation flow 
	tempF = (float)FLOW_METER_COUNT;
	tempF = tempF * 0.037F;
	// Clear flow meter count
	FLOW_METER_COUNT = 0;

	return (uint32_t)tempF;
}

void EXTI9_5_IRQHandler(void)
{
	__HAL_GPIO_EXTI_CLEAR_IT(HAL_SENSOR_INPUT_PIN);
  	FLOW_METER_COUNT++;
}

