/******************************************************************************

* File Name          : LTM022A69B.c
* Author             : Librae
* Last Modified Date : 08/12/2010
* Description        : This file provides the 
						LTM022A69B LCD related functions' declaration.

******************************************************************************/

#include "main.h"
#include "stm32l4xx_hal.h"

#include "LTM022A69B.h"
#include "LCD_lib.h"
#include "KOR_FONT.h"

extern SPI_HandleTypeDef hspi1;

void SPI_Write(uint16_t data)
{
	uint8_t dataBuf[2];
	dataBuf[0] = (uint8_t)(data >> 8);
	dataBuf[1] = (uint8_t)(data);
	HAL_SPI_Transmit(&hspi1, dataBuf, 2, 1000);	
}

void LCD_WRITE_REG(uint16_t index)
{
	LCD_RS_L(); LCD_CS_L();	
	SPI_Write(index);
	LCD_CS_H(); LCD_RS_H();
}

void LCD_SEND_COMMAND(uint16_t index, uint16_t data)
{
	LCD_RS_L(); LCD_CS_L();
	SPI_Write(index);
	LCD_CS_H();
	
	LCD_RS_H(); LCD_CS_L();
	SPI_Write(data);
	LCD_CS_H();
}

void LCD_WRITE_COMMAND(uint16_t index, uint16_t data)
{
	LCD_RS_L();	LCD_CS_L();
	SPI_Write(index);
	// LCD_CS_H();
	LCD_RS_H();
		
	// LCD_CS_L();
	SPI_Write(data);
	LCD_CS_H();
}

void LCD_WRITE_DATA(uint16_t data)
{
	SPI_Write(data);
}

void lcd_clear_screen(uint16_t color_background)
{
	uint16_t i, j;

	LCD_WRITE_COMMAND(0x210,0x00);
	LCD_WRITE_COMMAND(0x211,0xEF);
	LCD_WRITE_COMMAND(0x212,0x0000);
	LCD_WRITE_COMMAND(0x213,0x013F);	
	LCD_WRITE_COMMAND(0x200,0x0000);
	LCD_WRITE_COMMAND(0x201,0x0000);
	
	LCD_WRITE_REG(0x202);	//RAM Write index
	
	LCD_RS_H();	LCD_CS_L();
	
	for(i=0;i<LCD_Y_SIZE;i++)
	{
		for(j=0;j<LCD_X_SIZE;j++)
		{
			LCD_WRITE_DATA(color_background);
		}
	}	
	// LCD_RS_L();
	LCD_CS_H();
}

void lcd_init(void)
{
	LCD_CS_H(); LCD_RS_H();
	
	//  Reset LCD	
	LCD_RST_L(); HAL_Delay(100);
	LCD_RST_H(); HAL_Delay(10);	


//	LCD_WRITE_COMMAND( 0x003, 0x0001 );
	LCD_WRITE_COMMAND( 0x03A, 0x0001 ); /* oschilliation start */
	HAL_Delay(100);
	/* Power settings */  	
	LCD_WRITE_COMMAND( 0x100, 0x0000 ); /*power supply setup*/	
	LCD_WRITE_COMMAND( 0x101, 0x0000 ); 
	LCD_WRITE_COMMAND( 0x102, 0x3110 ); 
	LCD_WRITE_COMMAND( 0x103, 0xe200 ); 
	LCD_WRITE_COMMAND( 0x110, 0x009d ); 
	LCD_WRITE_COMMAND( 0x111, 0x0022 ); 
	LCD_WRITE_COMMAND( 0x100, 0x0120 ); 
	HAL_Delay(20);

	LCD_WRITE_COMMAND( 0x100, 0x3120 );
	HAL_Delay(80);
	/* Display control */   
	LCD_WRITE_COMMAND( 0x001, 0x0100 );
	LCD_WRITE_COMMAND( 0x002, 0x0000 );

	LCD_WRITE_COMMAND( 0x003, 0x1230 );
//	LCD_WRITE_COMMAND( 0x003, 0x1238 );


	LCD_WRITE_COMMAND( 0x006, 0x0000 );
	LCD_WRITE_COMMAND( 0x007, 0x0101 );
	LCD_WRITE_COMMAND( 0x008, 0x0808 );
	LCD_WRITE_COMMAND( 0x009, 0x0000 );
	LCD_WRITE_COMMAND( 0x00b, 0x0000 );
	LCD_WRITE_COMMAND( 0x00c, 0x0000 );
	LCD_WRITE_COMMAND( 0x00d, 0x0018 );
	/* LTPS control settings */   
	LCD_WRITE_COMMAND( 0x012, 0x0000 );
	LCD_WRITE_COMMAND( 0x013, 0x0000 );
	LCD_WRITE_COMMAND( 0x018, 0x0000 );
	LCD_WRITE_COMMAND( 0x019, 0x0000 );

	LCD_WRITE_COMMAND( 0x203, 0x0000 );
	LCD_WRITE_COMMAND( 0x204, 0x0000 );

	LCD_WRITE_COMMAND( 0x210, 0x0000 );
	LCD_WRITE_COMMAND( 0x211, 0x00ef );
	LCD_WRITE_COMMAND( 0x212, 0x0000 );
	LCD_WRITE_COMMAND( 0x213, 0x013f );
	LCD_WRITE_COMMAND( 0x214, 0x0000 );
	LCD_WRITE_COMMAND( 0x215, 0x0000 );
	LCD_WRITE_COMMAND( 0x216, 0x0000 );
	LCD_WRITE_COMMAND( 0x217, 0x0000 );

	// Gray scale settings
	LCD_WRITE_COMMAND( 0x300, 0x5343);
	LCD_WRITE_COMMAND( 0x301, 0x1021);
	LCD_WRITE_COMMAND( 0x302, 0x0003);
	LCD_WRITE_COMMAND( 0x303, 0x0011);
	LCD_WRITE_COMMAND( 0x304, 0x050a);
	LCD_WRITE_COMMAND( 0x305, 0x4342);
	LCD_WRITE_COMMAND( 0x306, 0x1100);
	LCD_WRITE_COMMAND( 0x307, 0x0003);
	LCD_WRITE_COMMAND( 0x308, 0x1201);
	LCD_WRITE_COMMAND( 0x309, 0x050a);

	/* RAM access settings */ 
	LCD_WRITE_COMMAND( 0x400, 0x4027 );
	LCD_WRITE_COMMAND( 0x401, 0x0000 );
	LCD_WRITE_COMMAND( 0x402, 0x0000 );	/* First screen drive position (1) */   	
	LCD_WRITE_COMMAND( 0x403, 0x013f );	/* First screen drive position (2) */   	
	LCD_WRITE_COMMAND( 0x404, 0x0000 );

	LCD_WRITE_COMMAND( 0x200, 0x0000 );
	LCD_WRITE_COMMAND( 0x201, 0x0000 );
	
	LCD_WRITE_COMMAND( 0x100, 0x7120 );
	LCD_WRITE_COMMAND( 0x007, 0x0103 );
	HAL_Delay(10);
	LCD_WRITE_COMMAND( 0x007, 0x0113 );	
}

/******************************************************************************
* Function Name  : lcd_clear_area
* Description    : clear area,fill
* Input          : color_front:draw a rectangle, fill with color_front
                   x:
                   y:
                   width: rectangle width
                   height:rectangle height
* Output         : None
* Return         : None
******************************************************************************/
void lcd_clear_area(uint16_t color_front,
										uint16_t x,
										uint16_t y,
										uint16_t width,
										uint16_t height)
{
	uint16_t i, j;

	LCD_WRITE_COMMAND( 0x210, x ); 	//x start point
	LCD_WRITE_COMMAND( 0x212, y ); 	//y start point
	LCD_WRITE_COMMAND( 0x211, x + width - 1 );	//x end point
	LCD_WRITE_COMMAND( 0x213, y + height - 1 );	//y end point
		
	LCD_WRITE_COMMAND( 0x200, x );
	LCD_WRITE_COMMAND( 0x201, y );

	LCD_RS_L();
	LCD_WRITE_REG(0x202);	//RAM Write index
	LCD_CS_L();

	for( i = 0; i < height; i++ )
	{
		for( j = 0; j < width; j++ )
		{
			LCD_WRITE_DATA( color_front );
		}
	}
	LCD_CS_H();
}



/******************************************************************************
* Function Name  : lcd_set_cursor
* Description    : Set cursor
* Input          : x, y
* Output         : None
* Return         : None
******************************************************************************/
void lcd_set_cursor(uint16_t x,
										uint16_t y)
{
	if( (x > 320) || (y > 240) )
	{
		return;
	}
	LCD_WRITE_COMMAND( 0x200, x );
	LCD_WRITE_COMMAND( 0x201, y );
}

/******************************************************************************
* Function Name  : lcd_display_char
* Description    : ch_asc: ascii code of data. position_x, position_y.
					color_front, color_background.
* Input          : None
* Output         : None
* Return         : None
******************************************************************************/
void lcd_display_char(uint16_t ch_asc,
											uint16_t color_front,
											uint16_t color_background,
											uint16_t postion_x,
											uint16_t postion_y)
{
	uint16_t i, j, b;
	const uint8_t *p = 0;
	
	LCD_WRITE_COMMAND(0x210,postion_x * 8); 	//x start point
	LCD_WRITE_COMMAND(0x212,postion_y * 16); 	//y start point
	LCD_WRITE_COMMAND(0x211,postion_x * 8+7);	//x end point
	LCD_WRITE_COMMAND(0x213,postion_y * 16+15);	//y end point

	LCD_WRITE_COMMAND(0x200,postion_x * 8);
	LCD_WRITE_COMMAND(0x201,postion_y * 16);
		
	LCD_RS_L();
	LCD_WRITE_REG(0x202);	//RAM Write index


	LCD_CS_L();

	p = ascii;
	p += ch_asc * 16;

	for(j=0; j<16; j++)
	{
		b = *(p+j);

		for(i=0; i<8; i++)
		{
			if(b & 0x80)
			{
				LCD_WRITE_DATA(color_front);
			}
			else
			{
				LCD_WRITE_DATA(color_background);
			}
			b = b << 1;
		}
	}
	LCD_CS_H();
}


/******************************************************************************
* Function Name  : lcd_display_string
* Description    : *str: address of string data. 
					x: the xth row(0~30).
					y: the yth column(0~20).
					color_front, color_background.
* Input          : None
* Output         : None
* Return         : None
******************************************************************************/
void lcd_display_string(char *str, 
												uint16_t color_front, 
												uint16_t color_background, 
												uint16_t x, 
												uint16_t y)
{
	while (*str) 
	{ 
		lcd_display_char( *str, color_front, color_background, x, y);
		if(++x >= 30)
		{
			x=0;
			if(++y >= 20)
			{
				y=0;
			}
		}
		str ++;
	}
}

void lcd_display_GB2312(uint8_t gb, 
												uint16_t color_front, 
												uint16_t color_background, 
												uint16_t postion_x, 
												uint16_t postion_y)
{
	uint16_t i, j, b;
	uint8_t *p;
	
	LCD_WRITE_COMMAND(0x210,postion_x*8); 	//x start point
	LCD_WRITE_COMMAND(0x212,postion_y*16); 	//y start point
	LCD_WRITE_COMMAND(0x211,postion_x*8+7);	//x end point
	LCD_WRITE_COMMAND(0x213,postion_y*16+15);	//y end point

	LCD_WRITE_COMMAND(0x200,postion_x*8);	
	LCD_WRITE_COMMAND(0x201,postion_y*16);

	LCD_RS_L();
	LCD_WRITE_REG(0x202);	//RAM Write index
	LCD_CS_L();

	//p = (unsigned char *) GB2312;
	p += gb * 32;
	for(j=0; j<32; j++)
	{
		b = *(p+j);
		for(i=0; i<8; i++)
		{
			if(b&0x80)
			{
				LCD_WRITE_DATA(color_front);
			}
			else
			{
				LCD_WRITE_DATA(color_background);
			}
			b = b << 1;
		}	
	}
	LCD_CS_H();
}

/******************************************************************************
* Function Name  : lcd_display_image
* Description    : Draw image
* Input          : x, y: image start at x, y. width, length, *img.
* Output         : None
* Return         : None
******************************************************************************/
void lcd_display_image(const char *img, 
												uint16_t x, 
												uint16_t y, 
												uint16_t width, 
												uint16_t height)
{
	uint16_t i, j;
	uint16_t data16;

	LCD_WRITE_COMMAND( 0x210, x ); 	//x start point
	LCD_WRITE_COMMAND( 0x212, y ); 	//y start point
	LCD_WRITE_COMMAND( 0x211, x + width - 1 );	//x end point
	LCD_WRITE_COMMAND( 0x213, y + height - 1 );	//y end point
		
	LCD_WRITE_COMMAND( 0x200, x );
	LCD_WRITE_COMMAND( 0x201, y );

	LCD_RS_L();
	LCD_WRITE_REG(0x202);	//RAM Write index
	LCD_CS_L();

	for(i = 0; i < height; i++)
	{
		for(j = 0; j < width; j++)
		{
			//Be carful of MCU type, big endian or little endian
			//little endian
			data16 = ( *(img + 1) << 8 ) | (*img);
			LCD_WRITE_DATA( data16 );
			img += 2;
		}
	}
	LCD_CS_H();
}

void Swap(uint16_t *a, uint16_t *b)	//for BresenhamLine
{
	uint16_t tmp;
	tmp = *a ;
	*a = *b ;
	*b = tmp ;
} 

/******************************************************************************
* Function Name  : lcd_draw_dot
* Description    : draw dot
* Input          : color, x, y
* Output         : None
* Return         : None
******************************************************************************/
void lcd_draw_dot(uint16_t color_front,
                  uint16_t x,
                  uint16_t y)
{

	LCD_SEND_COMMAND(0x210,x);
	LCD_SEND_COMMAND(0x212,y);
	LCD_SEND_COMMAND(0x211,x+1);
	LCD_SEND_COMMAND(0x213,y+1);	
	
	LCD_RS_L();
	LCD_WRITE_REG(0x202);	//RAM Write index
	LCD_CS_L();
	LCD_WRITE_DATA(color_front);
}

/******************************************************************************
* Function Name  : lcd_draw_bigdot
* Description    : draw big dot,9 pix.
* Input          : color_frong, x, y
* Output         : None
* Return         : None
******************************************************************************/
void lcd_draw_bigdot(uint16_t color_front,
                     uint16_t x,
                     uint16_t y)
{
    lcd_draw_dot(color_front,x,y);
    lcd_draw_dot(color_front,x,y+1);
    lcd_draw_dot(color_front,x,y-1);

    lcd_draw_dot(color_front,x+1,y);
    lcd_draw_dot(color_front,x+1,y+1);
    lcd_draw_dot(color_front,x+1,y-1);
    
    lcd_draw_dot(color_front,x-1,y);    
    lcd_draw_dot(color_front,x-1,y+1);
    lcd_draw_dot(color_front,x-1,y-1);    
}

/******************************************************************************
* Function Name  : lcd_draw_line
* Description    : BresenhamLine
* Input          : c, x1, y1, x2, y2
* Output         : None
* Return         : None
******************************************************************************/
uint8_t lcd_draw_line(uint16_t line_color,
										uint16_t x1,
										uint16_t y1,
										uint16_t x2,
										uint16_t y2)
{
	uint16_t dx, dy;
	uint16_t tx, ty;
	uint16_t inc1, inc2;
	int d, iTag;
	uint16_t x, y;
	
	lcd_draw_dot( line_color , x1 , y1 );
	if( x1 == x2 && y1 == y2 )
	{
		return 1;
	}
	
	iTag = 0;
	dx = ( x2 - x1 );
	dy = ( y2 - y1 );
	if( dx < dy )
	{
		iTag = 1 ;
		Swap ( &x1, &y1 );
		Swap ( &x2, &y2 );
		Swap ( &dx, &dy );
	}
	tx = ( x2 - x1 ) > 0 ? 1 : -1;
	ty = ( y2 - y1 ) > 0 ? 1 : -1;
	x = x1;
	y = y1;
	inc1 = 2 * dy;
	inc2 = 2 * ( dy - dx );
	d = inc1 - dx ;
	
	while( x != x2 )
	{
		if( d < 0 )
		{
			d += inc1 ;
		}
		else
		{
			y += ty ;
			d += inc2 ;
		}
		if( iTag )
		{
			lcd_draw_dot ( line_color, y, x ) ;
		}
		else
		{
			lcd_draw_dot ( line_color, x, y ) ;
		}
		x += tx ;
	}
	return 0;
}

uint64_t mypow(uint16_t m,uint16_t n)
{
	uint64_t result=1;	 
	while(n--)result*=m;    
	return result;
}

/**********************************************/
void lcd_display_number(uint16_t x,
                        uint16_t y,
                        uint64_t num,
                        uint16_t num_len)
{         	
	uint16_t t, temp;
	uint16_t enshow = 0;
	
	for(t=0;t<num_len;t++)
	{
		temp=(num/mypow(10,num_len-t-1))%10;
		if(enshow==0&&t<(num_len-1))
		{
			if(temp == 0)
			{
        lcd_display_char(' ',BLACK,WHITE,x+t,y);
				continue;
			}
			else
			{
					enshow = 1; 
			}		 	 
		}
    lcd_display_char(temp+'0',BLACK,WHITE,x+t,y); 
	}
}



void lcd_display_kor_char(
											uint8_t* pKorbuf,
											uint16_t color_front,
											uint16_t color_background,
											uint16_t postion_x,
											uint16_t postion_y)
{
	uint16_t i, j, b, b1;
	const uint8_t *p = 0;

	LCD_WRITE_COMMAND(0x210,postion_x * 8); 	//x start point
	LCD_WRITE_COMMAND(0x211,postion_x * 8+15);	//x end point
	LCD_WRITE_COMMAND(0x212,postion_y * 16); 	//y start point
	LCD_WRITE_COMMAND(0x213,postion_y * 16+15);	//y end point

	LCD_WRITE_COMMAND(0x200,postion_x * 8);
	LCD_WRITE_COMMAND(0x201,postion_y * 16);

	LCD_RS_L();
	LCD_WRITE_REG(0x202);	//RAM Write index

	LCD_CS_L();

	p = pKorbuf;

	for(j=0; j<16; j++)
	{
		b = *(p+j);
		for(i=0; i<8; i++)
		{
			if(b & 0x01)
			{
				LCD_WRITE_DATA(color_front);
			}
			else
			{
				LCD_WRITE_DATA(color_background);
			}
			b = b >> 1;
		}

		b1 = *(p+j+16);
		for(i=0; i<8; i++)
		{
			if(b1 & 0x01)
			{
				LCD_WRITE_DATA(color_front);
			}
			else
			{
				LCD_WRITE_DATA(color_background);
			}
			b1 = b1 >> 1;
		}
	}
	LCD_CS_H();
}

void lcd_display_eng_char(
											uint8_t* pEngbuf,
											uint16_t color_front,
											uint16_t color_background,
											uint16_t postion_x,
											uint16_t postion_y)
{
	uint16_t i, j, b, b1;
	const uint8_t *p = 0;

	LCD_WRITE_COMMAND(0x210, postion_x * 8); 	    //x start point
	LCD_WRITE_COMMAND(0x212, postion_y * 16); 	  //y start point
//	LCD_WRITE_COMMAND(0x211, postion_x * 8+15);	  //x end point
//	LCD_WRITE_COMMAND(0x213, postion_y * 16+7);	//y end point
	LCD_WRITE_COMMAND(0x211, postion_x * 8+7);	  //x end point
	LCD_WRITE_COMMAND(0x213, postion_y * 16+15);	//y end point

	LCD_WRITE_COMMAND(0x200, postion_x * 8);
	LCD_WRITE_COMMAND(0x201, postion_y * 16);
//	LCD_WRITE_COMMAND(0x200, postion_x * 16);
//	LCD_WRITE_COMMAND(0x201, postion_y * 8);

	LCD_RS_L();
	LCD_WRITE_REG(0x202);	//RAM Write index

	LCD_CS_L();

	p = pEngbuf;

	for(j=0; j<8; j++)
	{
		b = *(p+j);
		for(i=0; i<8; i++)
		{
			if(b & 0x01)
			{
				LCD_WRITE_DATA(color_front);
			}
			else
			{
				LCD_WRITE_DATA(color_background);
			}
			b = b >> 1;
		}

		b1 = *(p+j+8);
		for(i=0; i<8; i++)
		{
			if(b1 & 0x01)
			{
				LCD_WRITE_DATA(color_front);
			}
			else
			{
				LCD_WRITE_DATA(color_background);
			}
			b1 = b1 >> 1;
		}
	}

	LCD_CS_H();
}

uint16_t KSCodeConversion(uint16_t KSSM)	/* convert KSSM(완성형) to KS(조합형) */
{
	uint8_t HB, LB;
	uint16_t index, KS;

	HB = KSSM >> 8;
	LB = KSSM & 0x00FF;

	if(KSSM >= 0xB0A1 && KSSM <= 0xC8FE)
	{
		index = (HB - 0xB0)*94 + LB - 0xA1;
		KS  =  KS_Table[index][0] * 256;
		KS |=  KS_Table[index][1];
		return KS;
	}
	return -1;
}

void Korean(uint16_t nRow,
						uint16_t nCol,
						uint16_t code,
						uint16_t color_front,
						uint16_t color_background)			/* write a Korean character */
{
	unsigned char cho_5bit, joong_5bit, jong_5bit;
	unsigned char cho_bul, joong_bul, jong_bul = 0, i, jong_flag;
	unsigned int ch;
	unsigned char Kbuffer[32] = {0};		// 32 byte Korean font buffer
	unsigned char x;

	cho_5bit   = table_cho[(code >> 10) & 0x001F];	// get 5bit(14-10) of chosung
	joong_5bit = table_joong[(code >> 5) & 0x001F];	// get 5bit(09-05) of joongsung
	jong_5bit  = table_jong[code & 0x001F];					// get 5bit(04-00) of jongsung

	if(jong_5bit == 0)	// if jongsung not exist
	{
		jong_flag = 0;
		cho_bul = bul_cho1[joong_5bit];

		if((cho_5bit == 1) || (cho_5bit == 16)) joong_bul = 0;
		else
			joong_bul = 1;
	}
	else				// if jongsung exist
	{
		jong_flag = 1;
		cho_bul = bul_cho2[joong_5bit];

		if((cho_5bit == 1) || (cho_5bit == 16)) joong_bul = 2;
		else
			joong_bul = 3;

		jong_bul = bul_jong[joong_5bit];
	}

	ch = cho_bul*20 + cho_5bit;			// get chosung font
	for(i = 0; i < 32; i++) Kbuffer[i] = K_font[ch][i];

	ch = 8*20 + joong_bul*22 + joong_5bit;	// OR joongsung font
	for(i = 0; i < 32; i++) Kbuffer[i] |= K_font[ch][i];

	if(jong_flag)					// OR jongsung font
	{
		ch = 8*20 + 4*22 + jong_bul*28 + jong_5bit;
		for(i = 0; i < 32; i++)
			Kbuffer[i] |= K_font[ch][i];
	}

	lcd_display_kor_char(Kbuffer, color_front, color_background, nCol, nRow);
}


void English(	uint16_t nRow,
							uint16_t nCol,
							uint16_t code,
							uint16_t color_front,
							uint16_t color_background)			/* write a English ASCII character */
{
	unsigned char i;
	unsigned char Kbuffer[16] = {0};

	for(i = 0; i < 16; i++)
	{
		Kbuffer[i] = E_font[code][i];
	}
	lcd_display_eng_char(Kbuffer, color_front, color_background, nCol, nRow);
}


void lcd_string(uint16_t nLow,
								uint16_t nCol,
								char* str,
								uint16_t color_front,
								uint16_t color_background)
{
	uint8_t ch1;
	uint16_t ch2;

	while(*str) {
		ch1 = *str;
		str++;

		if(ch1 < 0x80) // English ASCII character
		{
			English(nLow, nCol, ch1, color_front, color_background);
//			English(nCol, nLow, ch1, color_front, color_background);
			nCol++;
		}
		else // Korean
		{
			ch2 = (ch1 << 8) + (*str);
			str ++;
			ch2 = KSCodeConversion(ch2);	// convert KSSM(완성형) to KS(조합형)
			Korean(nLow, nCol, ch2, color_front, color_background);
//			Korean(nCol, nLow, ch2, color_front, color_background);
			nCol+=2;
		}
	}
}

/********************************end of file**************************************/


