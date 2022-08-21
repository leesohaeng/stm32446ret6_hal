#include "Tps62191Handle.h"

/* Private define ------------------------------------------------------------*/
#define TPS62191_BYPASS_GPIO    GPIOA
#define TPS62191_BYPASS_PIN     GPIO_PIN_15
/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

void TPS61291BypassDisable(void)
{
	// TPS62191 레귤레이터의 EN/BYP PIN을 HIGH로 SET 하여야 Boost 모드로 동작한다.
	GPIO_InitTypeDef  GPIO_InitStruct;
    // Handle gpio clock enable
	__HAL_RCC_GPIOA_CLK_ENABLE();
	// Tps62191 bypass gpio initialize
	GPIO_InitStruct.Pin = TPS62191_BYPASS_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(TPS62191_BYPASS_GPIO , &GPIO_InitStruct);
	
    // Set tps62191 bypass gpio 
	HAL_GPIO_WritePin(TPS62191_BYPASS_GPIO, TPS62191_BYPASS_PIN, GPIO_PIN_SET);

    return;
}