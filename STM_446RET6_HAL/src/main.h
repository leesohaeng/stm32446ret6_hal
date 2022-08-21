/*
 * main.h
 *
 *  Created on: Feb 21, 2016
 *      Author: sohae
 */

#ifndef MAIN_H_
#define MAIN_H_

#endif /* MAIN_H_ */

/* USER CODE BEGIN Includes */
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE END Includes */

void 		mDelay();
void 		Delay(__IO uint32_t nCount);
