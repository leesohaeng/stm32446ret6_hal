/************************ (C) COPYLEFT 2010 Leafgrass *************************

* File Name          : LTM022A69B.h
* Author             : Librae
* Last Modified Date : 08/10/2010
* Description        : This file provides the 
						LTM022A69B LCD related functions' declaration.

******************************************************************************/
#ifndef __LTM022A69B_H__
#define __LTM022A69B_H__

#include <stdint.h>

//if IO for LCD is to be changed, just modify the constants below
#define LCD_X_SIZE		320	//LCD width
#define LCD_Y_SIZE		240	//LCD height

#define LCD_RST_GPIO_Port GPIOA
#define LCD_RS_GPIO_Port  GPIOA
#define LCD_CS_GPIO_Port  GPIOA

#define LCD_RST_Pin 	  GPIO_PIN_6
#define LCD_RS_Pin 	  	  GPIO_PIN_5
#define LCD_CS_Pin	 	  GPIO_PIN_4

//color define
#define CYAN		 0x07FF
#define PURPLE		 0xF81F
#define RED			 0XF800
#define GREEN        0X07E0
#define BLUE         0X001F
#define WHITE        0XFFFF
#define BLACK        0X0000
#define YELLOW       0XFFE0
#define ORANGE       0XFC08
#define GRAY  	     0X8430
#define LGRAY        0XC618
#define DARKGRAY     0X8410
#define PORPO        0X801F
#define PINK         0XF81F
#define GRAYBLUE     0X5458
#define LGRAYBLUE    0XA651
#define DARKBLUE     0X01CF
#define LIGHTBLUE    0X7D7C

#define LCD_RST_H() HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET)		// Set RST to 1
#define LCD_RST_L() HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET)	// Set RST to 0

#define LCD_RS_H() HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET) 			// Set RS to 1
#define LCD_RS_L() HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET) 		// Set RS to 0

#define LCD_CS_H() HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET) 			// Set CS to 1
#define LCD_CS_L() HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET) 		// Set CS to 0


//=============================================================================
//							LCD Application Functions
//=============================================================================
void LCD_WRITE_REG(uint16_t index);
void LCD_SEND_COMMAND(uint16_t index, uint16_t data);
void LCD_WRITE_COMMAND(uint16_t index, uint16_t data);
void LCD_WRITE_DATA(uint16_t data);


void lcd_init(void);
void lcd_clear_screen(uint16_t color_background);


void lcd_clear_area(uint16_t color_front,
										uint16_t x,
										uint16_t y,
										uint16_t width,
										uint16_t height);

void lcd_set_cursor(uint16_t x,
										uint16_t y);

void lcd_display_char(uint16_t ch_asc,
											uint16_t color_front,
											uint16_t color_background,
											uint16_t postion_x,
											uint16_t postion_y);

void lcd_display_string(char *str, 
												uint16_t color_front, 
												uint16_t color_background, 
												uint16_t x, 
												uint16_t y );


void lcd_display_GB2312(uint8_t gb, 
												uint16_t color_front, 
												uint16_t color_background, 
												uint16_t postion_x, 
												uint16_t postion_y);

void lcd_display_image(const char *img, 
												uint16_t x, 
												uint16_t y, 
												uint16_t width, 
												uint16_t height);

void lcd_draw_dot(uint16_t color_front,
                  uint16_t x,
                  uint16_t y);


void lcd_draw_bigdot(uint16_t color_front,
                     uint16_t x,
                     uint16_t y);

uint8_t lcd_draw_line(uint16_t line_color,
										uint16_t x1,
										uint16_t y1,
										uint16_t x2,
										uint16_t y2);

void lcd_display_number(uint16_t x,
                        uint16_t y,
                        uint64_t num,
                        uint16_t num_len);


void lcd_string(uint16_t nLow, 
								uint16_t nCol, 
								char* str,
								uint16_t color_front,
								uint16_t color_background);


#endif
