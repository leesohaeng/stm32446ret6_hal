
#ifndef SSD1306_H
#define SSD1306_H

/* C++ detection */
#ifdef __cplusplus
extern C {
#endif

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "fonts.h"

/**
 * This Library is written and optimized by Olivier Van den Eede(4ilo) in 2016
 * for Stm32 Uc and HAL-i2c lib's.
 *
 * To use this library with ssd1306 oled display you will need to customize the defines below.
 *
 * This library uses 2 extra files (fonts.c/h).
 * In this files are 3 different fonts you can use:
 * 		- Font_7x10
 * 		- Font_11x18
 * 		- Font_16x26
 *
 */

// Address for 128x32 is 0x3C
// Address for 128x64 is 0x3D (default) or 0x3C (if SA0 is grounded)

/*=========================================================================
    SSD1306 Displays
    -----------------------------------------------------------------------
    The driver is used in multiple displays (128x64, 128x32, etc.).
    Select the appropriate display below to create an appropriately
    sized framebuffer, etc.
    SSD1306_128_64  128x64 pixel display
    SSD1306_128_32  128x32 pixel display
    SSD1306_96_16
SSD1306    |STM32F10x    |DESCRIPTION

VCC        |3.3V         |
GND        |GND          |
SCL        |PB6          |Serial clock line
SDA        |PB7          |Serial data line
    -----------------------------------------------------------------------*/
#define SSD1306_128_64
// #define SSD1306_128_32
// #define SSD1306_96_16

//#define _SSD1306_I2C_DMA_
 #define _SSD1306_I2C_
/*=========================================================================*/


#if defined SSD1306_128_64
#define SSD1306_WIDTH                  128
#define SSD1306_HEIGHT                 64
#define SSD1306_BUFFER_SIZE            1024
#endif
#if defined SSD1306_128_32
#define SSD1306_WIDTH                  128
#define SSD1306_HEIGHT                 32
#define SSD1306_BUFFER_SIZE            512
#endif
#if defined SSD1306_96_16
#define SSD1306_WIDTH                  96
#define SSD1306_HEIGHT                 16
#define SSD1306_BUFFER_SIZE            192
#endif

/* I2C address */
#ifndef SSD1306_I2C_ADDR
#define SSD1306_I2C_ADDR        0x78
//#define SSD1306_I2C_ADDR       0x7A
#endif


//
//  Enumeration for screen colors
//
typedef enum {
	Black = 0x00, // Black color, no pixel
	White = 0x01  //Pixel is set. Color depends on LCD
} SSD1306_COLOR;

//
//  Struct to store transformations
//
typedef struct {
	I2C_HandleTypeDef *  hi2c;
	uint16_t CurrentX;
	uint16_t CurrentY;
	uint8_t Inverted;
	uint8_t Initialized;
} SSD1306_t;

//	Definition of the i2c port in main
// extern I2C_HandleTypeDef SSD1306_I2C_PORT;

//
//  Function definitions
//
uint8_t ssd1306_Init(I2C_HandleTypeDef * hi2c);
void ssd1306_Fill(SSD1306_COLOR color);
void ssd1306_UpdateScreen(void);
__STATIC_INLINE void ssd1306_ClearScreen(void) { ssd1306_Fill(Black); }
void ssd1306_ToggleInvert(void);
void ssd1306_DrawPixel(uint16_t x, uint16_t y, SSD1306_COLOR color);
char ssd1306_WriteChar(char ch, FontDef* Font, SSD1306_COLOR color);
char ssd1306_WriteString(char* str, FontDef* Font, SSD1306_COLOR color);
void ssd1306_SetCursor(uint8_t x, uint8_t y);

#ifndef ssd1306_I2C_TIMEOUT
#define ssd1306_I2C_TIMEOUT					20000
#endif
/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
