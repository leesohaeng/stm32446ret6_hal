/* ============================================================================ */
/*     font.h : TFT-LCD Korean and English ASCII Font Header File	*/
/* ============================================================================ */
/*                          Designed and programmed by Duck-Yong Yoon in 2010.  */

#ifndef _KOR_FONT_H_
#define _KOR_FONT_H_

/* ---------------------------------------------------------------------------- */
/*		한글 폰트 처리 테이블						*/
/* ---------------------------------------------------------------------------- */
extern const unsigned char table_cho[21];
extern const unsigned char table_joong[30];
extern const unsigned char table_jong[30];
extern const unsigned char bul_cho1[22];
extern const unsigned char bul_cho2[22];
extern const unsigned char bul_jong[22];

/* ---------------------------------------------------------------------------- */
/*		한글 16x16 폰트 선택(명조체, 고딕체, 필기체)			*/
/* ---------------------------------------------------------------------------- */
#define    KFONT16_MYUNGJO		// 16x16 명조체 선택
//#define    KFONT16_GOTHIC			// 16x16 고딕체 선택
//#define    KFONT16_PILGI			// 16x16 필기체 선택

/* ---------------------------------------------------------------------------- */
/*			16x16 한글 명조체 폰트					*/
/* ---------------------------------------------------------------------------- */
#ifdef KFONT16_MYUNGJO
extern const unsigned char K_font[][32];
#endif

/* ---------------------------------------------------------------------------- */
/*			16x16 한글 고딕체 폰트					*/
/* ---------------------------------------------------------------------------- */
#ifdef KFONT16_GOTHIC
extern const unsigned char K_font[][32];
#endif

/* ---------------------------------------------------------------------------- */
/*			16x16 한글 필기체 폰트					*/
/* ---------------------------------------------------------------------------- */
#ifdef KFONT16_PILGI
extern const unsigned char K_font[][32];
#endif

/* ---------------------------------------------------------------------------- */
/*		완성형(KSSM) ---> 조합형(KS) 변환 테이블			*/
/* ---------------------------------------------------------------------------- */
extern const unsigned char KS_Table[][2];

/* ---------------------------------------------------------------------------- */
/*		8x16 영문 ASCII 폰트						*/
/* ---------------------------------------------------------------------------- */
extern const unsigned char E_font[128][16];

#endif
