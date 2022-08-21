/*
 * tlcd.h
 *
 *  Created on: Jan 5, 2016
 *      Author: LEE
 */

#ifndef TLCD_H_
#define TLCD_H_

#endif /* TLCD_H_ */

#define LOW		0
#define HIGH	1

typedef unsigned char BYTE;

void	TLCD_Init(GPIO_TypeDef *GPIOn, uint32_t RS,uint32_t E, uint32_t Data4,uint32_t Data5,uint32_t Data6,uint32_t Data7);
void	TLCD_4BitWrite(BYTE c);
void	TLCD_Enable();
void 	TLCD_Cmd(BYTE cmd);
void	TLCD_Write(BYTE v);
void	TNLD_Send(uint8_t v,uint8_t m);
void	TLCD_Putc(char c);
void	TLCD_Puts(uint8_t line, uint8_t n,char *s);
