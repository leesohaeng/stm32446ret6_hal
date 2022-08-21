#include "Lom202aHandle.h"
#include <string.h>

/* Private define ------------------------------------------------------------*/
// Lora comm serial gpio defines
#define LORA_COMM_SERIAL_GPIO       GPIOA
#define LORA_COMM_SERIAL_RX_PIN	    GPIO_PIN_10
#define LORA_COMM_SERIAL_TX_PIN	    GPIO_PIN_9
// Lora handle gpio defines
#define LORA_HANDLE_GPIO            GPIOB
#define LORA_HANDLE_RESET_PIN       GPIO_PIN_10
#define LORA_HANDLE_WAKEUP_PIN	    GPIO_PIN_11
#define LORA_HANDLE_UPDATE_PIN	    GPIO_PIN_12
// Lora comm serial recv buffer size
#define MAX_LORA_COMM_RECV_BUFF     (256)
/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    uint8_t BUFF[MAX_LORA_COMM_RECV_BUFF];
    uint16_t HEAD;
    uint16_t TAIL;
    uint16_t COUNT;
}LORA_COMM_RECV_BUFF;
/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
// Lora comm serial handle
UART_HandleTypeDef 	HANDLE_LORA_COMM_SERIAL;
// Lora receive buffer
LORA_COMM_RECV_BUFF LORA_RECV_BUFF;
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

void LoraHandleGpio_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
	// Lora handle gpio clock enable
    __HAL_RCC_GPIOB_CLK_ENABLE();
	// Lora reset gpio
	GPIO_InitStruct.Pin = 	LORA_HANDLE_RESET_PIN;
	GPIO_InitStruct.Mode = 	GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = 	GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(LORA_HANDLE_GPIO, &GPIO_InitStruct);
	// Lora wake up gpio
	GPIO_InitStruct.Pin = 	LORA_HANDLE_WAKEUP_PIN;
	GPIO_InitStruct.Mode = 	GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = 	GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(LORA_HANDLE_GPIO, &GPIO_InitStruct);
	// Lora update gpio
	GPIO_InitStruct.Pin = 	LORA_HANDLE_UPDATE_PIN;
	GPIO_InitStruct.Mode = 	GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = 	GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(LORA_HANDLE_GPIO, &GPIO_InitStruct);
    // Lora update gpio set low
	HAL_GPIO_WritePin(LORA_HANDLE_GPIO, LORA_HANDLE_UPDATE_PIN, GPIO_PIN_RESET);
    
    // Lora module default off
    LoraModuleOff();

    return;
}

void LoraModuleOff(void)
{
	HAL_GPIO_WritePin(LORA_HANDLE_GPIO, LORA_HANDLE_RESET_PIN, GPIO_PIN_RESET); 
    return;
}

void LoraModuleOn(void)
{
	HAL_GPIO_WritePin(LORA_HANDLE_GPIO, LORA_HANDLE_RESET_PIN, GPIO_PIN_SET);
    return;
}

void LoraReset(void)
{
	HAL_GPIO_WritePin(LORA_HANDLE_GPIO, LORA_HANDLE_RESET_PIN, GPIO_PIN_SET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(LORA_HANDLE_GPIO, LORA_HANDLE_RESET_PIN, GPIO_PIN_RESET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(LORA_HANDLE_GPIO, LORA_HANDLE_RESET_PIN, GPIO_PIN_SET);
    return;
}

void LoraWakeUp(void)
{
	HAL_GPIO_WritePin(LORA_HANDLE_GPIO, LORA_HANDLE_WAKEUP_PIN, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LORA_HANDLE_GPIO, LORA_HANDLE_WAKEUP_PIN, GPIO_PIN_SET);
	HAL_Delay(1);
    return;
}

void LoraCommSerial_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

	// Uart peripheral and gpio clock enable
    __HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();

    // Uart tx gpio
	GPIO_InitStruct.Pin     = LORA_COMM_SERIAL_TX_PIN;
	GPIO_InitStruct.Mode    = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull    = GPIO_NOPULL;
	GPIO_InitStruct.Speed   = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(LORA_COMM_SERIAL_GPIO, &GPIO_InitStruct);
	// Uart rx gpio
	GPIO_InitStruct.Pin     = LORA_COMM_SERIAL_RX_PIN;
	GPIO_InitStruct.Mode    = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull    = GPIO_NOPULL;
	GPIO_InitStruct.Speed   = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(LORA_COMM_SERIAL_GPIO, &GPIO_InitStruct);
    // Uart peripheral
	HANDLE_LORA_COMM_SERIAL.Instance 			= USART1;
	HANDLE_LORA_COMM_SERIAL.Init.BaudRate 		= 115200;
	HANDLE_LORA_COMM_SERIAL.Init.WordLength   	= UART_WORDLENGTH_8B;
	HANDLE_LORA_COMM_SERIAL.Init.StopBits     	= UART_STOPBITS_1;
	HANDLE_LORA_COMM_SERIAL.Init.Parity       	= UART_PARITY_NONE;
	HANDLE_LORA_COMM_SERIAL.Init.HwFlowCtl    	= UART_HWCONTROL_NONE;
	HANDLE_LORA_COMM_SERIAL.Init.Mode         	= UART_MODE_TX_RX;
	HANDLE_LORA_COMM_SERIAL.Init.OverSampling 	= UART_OVERSAMPLING_16;
	HAL_UART_Init(&HANDLE_LORA_COMM_SERIAL);
    // Uart receive interrupt initialize
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 2);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	__HAL_UART_ENABLE_IT(&HANDLE_LORA_COMM_SERIAL, UART_IT_RXNE);
    // Receive buffer flush
    LoraCommSerial_RxFlush();

    return;
}

void LoraCommSerial_RxFlush(void)
{
	memset(LORA_RECV_BUFF.BUFF, 0x0, sizeof(LORA_RECV_BUFF.BUFF));
    LORA_RECV_BUFF.COUNT = 0;
    LORA_RECV_BUFF.HEAD = 0;
    LORA_RECV_BUFF.TAIL = 0;
    return;
}

uint16_t LoraCommSerial_RxExist(void)
{
	return LORA_RECV_BUFF.COUNT;
}

uint8_t LoraCommSerial_GetByte(void)
{
	uint8_t getCh;
  
	if(LORA_RECV_BUFF.COUNT) {
	    getCh = LORA_RECV_BUFF.BUFF[LORA_RECV_BUFF.TAIL];
	    LORA_RECV_BUFF.COUNT --;
	    LORA_RECV_BUFF.TAIL = ((LORA_RECV_BUFF.TAIL + 1) % MAX_LORA_COMM_RECV_BUFF);
	}
    return getCh;
}

void LoraCommSerial_Send(uint8_t *pBuff, uint32_t buffLen)
{
    LoraWakeUp();
    HAL_Delay(2);
	HAL_UART_Transmit(&HANDLE_LORA_COMM_SERIAL, pBuff, buffLen, HAL_MAX_DELAY);
    return;
}

void USART1_IRQHandler(void)
{
	uint8_t getCh;
	  
	if(__HAL_UART_GET_FLAG(&HANDLE_LORA_COMM_SERIAL, UART_FLAG_RXNE) == SET) 
    {
		HAL_UART_Receive(&HANDLE_LORA_COMM_SERIAL, &getCh, 1, HAL_MAX_DELAY);
		
		LORA_RECV_BUFF.BUFF[LORA_RECV_BUFF.HEAD] = getCh;
		LORA_RECV_BUFF.HEAD = (LORA_RECV_BUFF.HEAD + 1) % MAX_LORA_COMM_RECV_BUFF;
		LORA_RECV_BUFF.COUNT ++;

		if(LORA_RECV_BUFF.COUNT >= MAX_LORA_COMM_RECV_BUFF) {
			LORA_RECV_BUFF.COUNT = MAX_LORA_COMM_RECV_BUFF;
			LORA_RECV_BUFF.HEAD = LORA_RECV_BUFF.TAIL;
		}
		__HAL_UART_CLEAR_FLAG(&HANDLE_LORA_COMM_SERIAL, UART_IT_RXNE);
	}
    return;
}
