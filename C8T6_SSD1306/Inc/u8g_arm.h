/*
 * u8g_arm.h
 *
 *  Created on: 2017. 10. 10.
 *      Author: sohaenglee
 */

 #ifndef _U8G_ARM_H
 #define _U8G_ARM_H
 #include "u8g.h"
 #include "stm32f1xx_hal.h"


// spi 사용을 위해서 반드시 고쳐야 함.
 #define SPI_HANDLER hspi1 // use your SPI hadler
 extern SPI_HandleTypeDef SPI_HANDLER;
#define  RES	GPIO_PIN_10
 #define DC	 	GPIO_PIN_9
 #define CS 	GPIO_PIN_8
 #define PORT GPIOA
 uint8_t u8g_com_hw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);
 #endif
