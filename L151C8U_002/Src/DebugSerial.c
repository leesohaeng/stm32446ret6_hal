#include "DebugSerial.h"\
#include <string.h>

/* Private define ------------------------------------------------------------*/
// Debug serial gpio defines
#define DEBUG_SERIAL_GPIO 		GPIOA
#define DEBUG_SERIAL_RX_PIN 	GPIO_PIN_3
#define DEBUG_SERIAL_TX_PIN 	GPIO_PIN_2
// Debug serial recv buffer size
#define MAX_DEBUG_RECV_BUFF     (256)
/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    uint8_t BUFF[MAX_DEBUG_RECV_BUFF];
    uint16_t HEAD;
    uint16_t TAIL;
    uint16_t COUNT;
}DEBUG_SERIAL_RECV_BUFF;
/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
// Debug serial handle
UART_HandleTypeDef 	HANDLE_DEBUG_SERIAL;
// Lora receive buffer
DEBUG_SERIAL_RECV_BUFF DEBUG_RECV_BUFF;
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/\
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif 
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&HANDLE_DEBUG_SERIAL, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}
/* Private function prototypes -----------------------------------------------*/

void DebugSerial_Init(void)
{
	static GPIO_InitTypeDef  GPIO_InitStruct;
	
    // Uart peripheral and gpio clock enable
    __HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();
	
	// Uart tx gpio
	GPIO_InitStruct.Pin = DEBUG_SERIAL_TX_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(DEBUG_SERIAL_GPIO, &GPIO_InitStruct);
	// Uart rx gpio
	GPIO_InitStruct.Pin = DEBUG_SERIAL_RX_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(DEBUG_SERIAL_GPIO, &GPIO_InitStruct);
	// Uart peripheral
	HANDLE_DEBUG_SERIAL.Instance 			= USART2;
	HANDLE_DEBUG_SERIAL.Init.BaudRate 		= 115200;
	HANDLE_DEBUG_SERIAL.Init.WordLength  	= UART_WORDLENGTH_8B;
	HANDLE_DEBUG_SERIAL.Init.StopBits     	= UART_STOPBITS_1;
	HANDLE_DEBUG_SERIAL.Init.Parity       	= UART_PARITY_NONE;
	HANDLE_DEBUG_SERIAL.Init.HwFlowCtl    	= UART_HWCONTROL_NONE;
	HANDLE_DEBUG_SERIAL.Init.Mode         	= UART_MODE_TX_RX;
	HANDLE_DEBUG_SERIAL.Init.OverSampling 	= UART_OVERSAMPLING_16;
	HAL_UART_Init(&HANDLE_DEBUG_SERIAL);
	// Uart receive interrupt initialize
	HAL_NVIC_SetPriority(USART2_IRQn, 1, 3);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	__HAL_UART_ENABLE_IT(&HANDLE_DEBUG_SERIAL, UART_IT_RXNE);
    // Receive buffer flush
    DebugSerial_RxFlush();
    return;
}

void DebugSerial_RxFlush(void)
{
	memset(DEBUG_RECV_BUFF.BUFF, 0x0, sizeof(DEBUG_RECV_BUFF.BUFF));
    DEBUG_RECV_BUFF.COUNT = 0;
    DEBUG_RECV_BUFF.HEAD = 0;
    DEBUG_RECV_BUFF.TAIL = 0;
    return;
}

uint16_t DebugSerial_RxExist(void)
{
	return DEBUG_RECV_BUFF.COUNT;
}

uint8_t DebugSerial_GetByte(void)
{
	uint8_t getCh;
  
	if(DEBUG_RECV_BUFF.COUNT) {
	    getCh = DEBUG_RECV_BUFF.BUFF[DEBUG_RECV_BUFF.TAIL];
	    DEBUG_RECV_BUFF.COUNT --;
	    DEBUG_RECV_BUFF.TAIL = ((DEBUG_RECV_BUFF.TAIL + 1) % MAX_DEBUG_RECV_BUFF);
	}
    return getCh;
}

void DebugSerial_Send(uint8_t *pBuff, uint32_t buffLen)
{
    HAL_UART_Transmit(&HANDLE_DEBUG_SERIAL, pBuff, buffLen, HAL_MAX_DELAY);
    return;
}

void USART2_IRQHandler(void)
{
	uint8_t getCh;
	  
	if(__HAL_UART_GET_FLAG(&HANDLE_DEBUG_SERIAL, UART_FLAG_RXNE) == SET) 
    {
		HAL_UART_Receive(&HANDLE_DEBUG_SERIAL, &getCh, 1, HAL_MAX_DELAY);

		DEBUG_RECV_BUFF.BUFF[DEBUG_RECV_BUFF.HEAD] = getCh;
		DEBUG_RECV_BUFF.HEAD = (DEBUG_RECV_BUFF.HEAD + 1) % MAX_DEBUG_RECV_BUFF;
		DEBUG_RECV_BUFF.COUNT++;

		if(DEBUG_RECV_BUFF.COUNT >= MAX_DEBUG_RECV_BUFF) {
			DEBUG_RECV_BUFF.COUNT = MAX_DEBUG_RECV_BUFF;
			DEBUG_RECV_BUFF.HEAD = DEBUG_RECV_BUFF.TAIL;
		}	
		__HAL_UART_CLEAR_FLAG(&HANDLE_DEBUG_SERIAL, UART_IT_RXNE);
	}
    return;	
}
