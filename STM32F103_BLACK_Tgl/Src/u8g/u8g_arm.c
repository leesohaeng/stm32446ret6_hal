/*
 * u8g_arm.c
 *
 *  Created on: 2017. 10. 10.
 *      Author: sohaenglee
 */

 #include "u8g_arm.h"
 #include "userDelay.h"
 void u8g_Delay(uint16_t val)
 {
      delay_ms(val);
 }
 void u8g_MicroDelay(void)
 {
	 delay_us(1);
 }
 void u8g_10MicroDelay(void)
 {
      delay_us(10);
 }
 uint8_t u8g_com_hw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr)
 {
  switch(msg)
  {
   case U8G_COM_MSG_STOP:
    break;
   case U8G_COM_MSG_INIT:
    u8g_MicroDelay();
    break;
   case U8G_COM_MSG_ADDRESS:           /* define cmd (arg_val = 0) or data mode (arg_val = 1) */
    u8g_10MicroDelay();
    HAL_GPIO_WritePin(PORT, DC, arg_val);
    u8g_10MicroDelay();
    break;
   case U8G_COM_MSG_CHIP_SELECT:
    if ( arg_val == 0 )
    {
    	delay_ms(1);
         HAL_GPIO_WritePin(PORT, CS, GPIO_PIN_SET);
    }
    else
      HAL_GPIO_WritePin(PORT, CS, GPIO_PIN_RESET);
      u8g_MicroDelay();
    break;
   case U8G_COM_MSG_RESET:
	      HAL_GPIO_WritePin(PORT, RES, GPIO_PIN_RESET);
	      delay_ms(1);
		  HAL_GPIO_WritePin(PORT, RES, GPIO_PIN_SET);
    break;
   case U8G_COM_MSG_WRITE_BYTE:
     HAL_SPI_Transmit(&SPI_HANDLER, &arg_val, 1, 10000);
     while(HAL_SPI_GetState(&SPI_HANDLER) != HAL_SPI_STATE_READY);
     u8g_MicroDelay();
     break;
   case U8G_COM_MSG_WRITE_SEQ:
   case U8G_COM_MSG_WRITE_SEQ_P:
    {
         HAL_SPI_Transmit(&SPI_HANDLER, (uint8_t *)arg_ptr, arg_val, 10000);
         while(HAL_SPI_GetState(&SPI_HANDLER) != HAL_SPI_STATE_READY);
         u8g_MicroDelay();
         arg_val = 0;
    }
    break;
  }
  return 1;
 }


