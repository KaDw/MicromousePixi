#ifndef __UI_H__
#define __UI_H__
//------------------------------------------------------------------//
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#define USE_FULL_ASSERT
#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "sensor.h"

#define LOG(m)

extern UART_HandleTypeDef huart1;
#define UI_UART_Handle huart1


#define UI_LED_L 			LED4_Pin
#define UI_LED_R 			LED3_Pin
#define UI_LED_GREEN	LED1_Pin
#define UI_LED_YELLOW	LED2_Pin

/* Buzzer music */
#define  ui_c     261    // 261 Hz 
#define  ui_d     294   // 294 Hz 
#define  ui_e     329    // 329 Hz 
#define  ui_f     349    // 349 Hz 
#define  ui_g     392    // 392 Hz 
#define  ui_a     440    // 440 Hz 
#define  ui_b     493    // 493 Hz 
#define  ui_C     523    // 523 Hz 

// _ftoa
#define MAX_PRECISION	(10)
static const double rounders[MAX_PRECISION + 1] =
{
	0.5,				// 0
	0.05,				// 1
	0.005,				// 2
	0.0005,				// 3
	0.00005,			// 4
	0.000005,			// 5
	0.0000005,			// 6
	0.00000005,			// 7
	0.000000005,		// 8
	0.0000000005,		// 9
	0.00000000005		// 10
};

void UI_Init(void);
void UI_InitBeep(void);
void UI_InitLeds(void);
void UI_InitBattControl(void);

void UI_Send(uint8_t* m);
void UI_Beep(int time, int freq);
void UI_LedOffAll(void);
void UI_LedOnAll(void);
void UI_LedOn(int UI_LED_n);
void UI_LedOff(int UI_LED_n);
void UI_LedToggle(int UI_LED_n);

void UI_BattControl(void);
int  UI_BattValue(void);

int32_t UI_Timestamp(void);
int32_t UI_TimeElapsedUs(int32_t timestamp);
void UI_DelayUs(int32_t us);
void UI_WaitBtnL(void);
void UI_WaitBtnR(void);

void UI_PrintData(void);
/*
+=============================================================================+
| printf_()
+=============================================================================+
*/

// called from printf_()
void usart_put_char(char ch);

struct FILE
{
	int handle;
};
/*
+=============================================================================+
| global definitions
+=============================================================================+
*/

#define PRINTF_HAVE_PRINTF					1				///< selects whether to compile printf_()
#define PRINTF_HAVE_SPRINTF					0				///< selects whether to compile sprintf_()
#define PRINTF_HAVE_PRINTF_SPRINTF			(PRINTF_HAVE_PRINTF && PRINTF_HAVE_SPRINTF)


#define stdout_								&stdout_file	///< stdout_ used by printf

/*
+=============================================================================+
| strange variables
+=============================================================================+
*/

/// a simplified FILE struct - only basic functionality which can be used by printf_()
typedef struct printf_file_s
{
#if PRINTF_HAVE_SPRINTF == 1
	char *buffer;							///< pointer to buffer for data
#endif

#if PRINTF_HAVE_PRINTF == 1
	void (*put)(char);						///< put() function for writing data
#endif

	int length;								///< user's variable for current length
} printf_file_t;

/*
+=============================================================================+
| global variables
+=============================================================================+
*/

/*
+=============================================================================+
| global functions' declarations
+=============================================================================+
*/

/// @brief convert int to char[]
/// @return return pointer to first char
char* _itoa(int num, char* buff, int base);
char* _ftoa(double f, char * buf, int precision);
/// @brief reverse string
/// @return pointer to first char
/// @param pointer to first char
/// @param pointer after last char
char* _strrev(char* s);

#if PRINTF_HAVE_PRINTF == 1
int printf_(const char *format, ...);
#endif

#if PRINTF_HAVE_SPRINTF == 1
int sprintf_(char *buffer, const char *format, ...);
#endif

/******************************************************************************
* END OF FILE
******************************************************************************/
#endif
