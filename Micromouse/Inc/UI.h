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

void UI_BattControl(void);


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
char* itoa(int num, char* buff, int base);

/// @brief reverse string
/// @return pointer to first char
/// @param pointer to first char
/// @param pointer after last char
char* strrev(char *begin, char *end);

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
