#include "stm32f4xx_hal.h"
#include "gpio_r.h"
#include "UI.h"
#include "usart.h"

void UI_InitDelayUs(void);

void UI_Init()
{
	MX_USART1_UART_Init();
	UI_InitBeep();
	UI_InitLeds();
	//UI_InitBattControl();
	UI_InitDelayUs();
	
	// pull up for buttons (PA15 PB2)
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR15_0; // pull-up
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR15_1; // pull-down
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR2_0; // pull-up
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR2_1; // pull-down
}


void UI_InitBeep()
{
	__TIM1_CLK_ENABLE();
	__TIM2_CLK_ENABLE();
	__GPIOA_CLK_ENABLE();
	__DSB(); // Data Synchronization Barrier
	
	gpio_pin_cfg(GPIOA, PA5, gpio_mode_AF1_PP_LS); // to jest ok
	
	
	//--- TIM1 master mode
	/*TIM1->CR1 |= TIM_CR1_DIR | TIM_CR1_OPM; // downcounter, one pulse mode
	//TIM1->CR2 = TIM_CR2_MMS_1; // CEN is used as TRGO
	TIM1->ARR = 5500; // duration*/
	// wlacz UEV interrupt to set TIM2->CNT = 0 albo 0xffff ==> celar SR_UIF
	//TIM1->DIER = TIM_DIER_UIE;
	//NVIC_EnableIRQ(TIM1_UP_TIM10_IRQHandler);
	
	//--- TIM2 CH1 config (freq)
	// select OCxM
	//TIM2->CNT = 5000; // (32bit)
	TIM2->CR1 &= ~TIM_CR1_CEN;
	TIM2->ARR = 65e3;//14e2; // 1k4 -> 60kHz (32bit) (max)
	TIM2->PSC = 65e3;//150; // current freq = 60kHz/150 = 400Hz
	//TIM2->CR1 = TIM_CR1_ARPE; // ARR shadow register
	TIM2->CCR2 = TIM2->ARR/2; // 50% duty
	TIM2->CCMR1 = TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_0; // PWM Mode 1 // toggle // CC2S=0:output
	TIM2->CCER = TIM_CCER_CC2E; // CC2E-output enable, CC1P - polarity
	TIM2->EGR = TIM_EGR_UG; // Update generation
	//TIM2->BDTR |= TIM_BDTR_MOE; // MOE-main output enable
	
	//TS=000 -> Master is TIM1
	//TS=001 -> Master is TIM8
	//TIM2->SMCR |= TIM_SMCR_SMS_2|TIM_SMCR_SMS_0; //TS=000 SMS=101 - gated mode
	
	// enable timers
	TIM2->CR1 |= TIM_CR1_CEN;
}

void UI_InitLeds()
{
	// config timers in One Pulse mode
	UI_LedOffAll();
}

void UI_InitBattControl()
{
	__TIM11_CLK_ENABLE();
	__DSB();// Data Synchronization Barrier
	
	// event generation register
	TIM11->EGR = TIM_EGR_UG; //  Update generation
	TIM11->DIER = TIM_DIER_UIE;
	// control register
	TIM11->PSC = 42000; //168e6/4e3 = 42000 mamy 4kHz zegar za prescalerem
	TIM11->ARR = 4000;
	
	NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
	
	TIM11->CR1 = TIM_CR1_CEN; //counter_enable
}


void UI_InitDelayUs()
{
	__TIM7_CLK_ENABLE();
	TIM7->PSC = 168/2;
	TIM7->EGR = TIM_EGR_UG;
	TIM7->CR1 = TIM_CR1_OPM;
}


void UI_Beep(int time, int freq)
{
	//freq to TIM2
	TIM2->PSC = 60e3 / freq + 1;
	
	//time to TIM1
	TIM1->CNT = time;
	TIM1->CR1 |= TIM_CR1_CEN;
}

void UI_LedOffAll()
{
 HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_SET);
 HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_SET);
 HAL_GPIO_WritePin(GPIOB, LED3_Pin, GPIO_PIN_SET);
 HAL_GPIO_WritePin(GPIOA, LED4_Pin, GPIO_PIN_SET);
}

void UI_LedOnAll()
{
 HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_RESET);
 HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);
 HAL_GPIO_WritePin(GPIOB, LED3_Pin, GPIO_PIN_RESET);
 HAL_GPIO_WritePin(GPIOA, LED4_Pin, GPIO_PIN_RESET);
}

void UI_LedOn(int UI_LED_n)
{
	if(UI_LED_n != LED4_Pin)
	{
		GPIOB->BSRR = UI_LED_n<<16;
	}
	else
	{
		GPIOA->BSRR = UI_LED_n<<16;
	}
}

void UI_LedOff(int UI_LED_n)
{
	if(UI_LED_n != LED4_Pin)
	{
		GPIOB->BSRR = UI_LED_n;
	}
	else
	{
		GPIOA->BSRR = UI_LED_n;
	}
}

void UI_BattControl()
{
	static uint32_t battValue = 0;
	static uint32_t lastValue = 0;
	
	if(battValue == 0)
		battValue = lastValue;
	else
		battValue = ((battValue << 2) + lastValue) / 5; // alpha-blending
	
	ADCreadChannel(CH9, &lastValue);
	 //1080LSB/2.7V
	if( 2200 < battValue && battValue < 2960 ) // 5.5V...7.4V
	{
		UI_Beep(500, 700);
		UI_LedOffAll();
		UI_LedOn(UI_LED_YELLOW);
	}
	
	//printf_("Batt:%d\n", battValue);
}

void UI_Send(uint8_t* m)
{
	HAL_UART_Transmit(&UI_UART_Handle, m, strlen((char*)m), 0xFF);
}

void UI_DelayUs(uint16_t us)
{
	TIM7->CNT = 1;
	TIM7->ARR = us;
	TIM7->CR1 |= TIM_CR1_CEN;
	while(TIM7->CNT != 0)
	{}
}

void UI_TimerUs(uint16_t us)
{
	TIM7->CNT = 1;
	TIM7->ARR = us;
	TIM7->CR1 |= TIM_CR1_CEN;
}

int UI_TimerBusy()
{
	return TIM7->CNT != 0;
}

void UI_WaitBtnL()
{
	int c = 0;
	while(c < 10000)
	{
		if(GPIOB->IDR & PB2)
			++c;
		else
			c = 0;
	}
}

void UI_WaitBtnR()
{
	int c = 0;
	while(c < 10000)
	{
	if(GPIOA->IDR & PA15)
		++c;
	else
		c=0;
	}
}



#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}




/*
+=============================================================================+
| printf_()
+=============================================================================+
*/

void usart_put_char(char ch)
{
	while (!(UI_UART_Handle.Instance->SR & UART_FLAG_TXE))
	{
	}
	
	UI_UART_Handle.Instance->DR = (ch & 0xFF);
}

/*
+=============================================================================+
| module variables
+=============================================================================+
*/

#if PRINTF_HAVE_PRINTF == 1

printf_file_t stdout_file = {
#if PRINTF_HAVE_PRINTF_SPRINTF == 1
		NULL,
#endif
		&usart_put_char, 0};	///< stdout file handle - usart_put_char() by default

#endif
/*
+=============================================================================+
| local functions' declarations
+=============================================================================+
*/

static int __fputc_(int character, printf_file_t *stream);
static int __vfprintf_(printf_file_t *stream, const char *format, va_list arg);

char* itoa(int num, char* buff, int base)
{
	char *ptr = buff;
	char minus = num < 0; // bool flag
	
	if(minus)
	{
		*buff++ = '-';
		
	}
	
	do
	{
		*ptr = num % base;
		*ptr += *ptr < 10 ? '0' : ('A'-10);
		++ptr;
		num /= base;
	}while(num);
	
	strrev(buff, ptr);
	*ptr = '\0';
	
	if(minus)
		return buff-1;
	else
		return buff;
}


char* strrev(char *begin, char *end)
{
	char* str = begin;
	char temp;
	--end;
	
	// reverse string
	while(begin < end)
	{
		temp = *end;
		*end = *begin;
		*begin = temp;
		++begin;
		--end;
	}
	
	return str;
}


#if PRINTF_HAVE_PRINTF == 1

/*------------------------------------------------------------------------*//**
* \brief Simplified printtf() - prints formatted string
* \details Prints a string to stdout_ (USART). Only %s, %d, %x, %c and %%
* specifiers are supported
*
* \param [in] format is a standard format string, additional parameters
* expected - one for each specifier
* \return number of written characters
*//*-------------------------------------------------------------------------*/

int printf_(const char *format, ...)
{
	va_list arg;
	int count;

	va_start(arg, format);
	count = __vfprintf_(stdout_, format, arg);
	va_end(arg);

	return count;
}

#endif

#if PRINTF_HAVE_SPRINTF == 1

/*------------------------------------------------------------------------*//**
* \brief Simplified sprinttf() - prints formatted string to buffer
* \details Prints a string to buffer. Only %s, %d, %x, %c and %% specifiers are
* supported
*
* \param [out] buffer is a buffer for string
* \param [in] format is a standard format string, additional parameters
* expected - one for each specifier
* \return number of written characters
*//*-------------------------------------------------------------------------*/

int sprintf_(char *buffer, const char *format, ...)
{
	printf_file_t stream;
	va_list arg;
	int count;

	stream.buffer = buffer;

#if PRINTF_HAVE_PRINTF == 1
	stream.put = NULL;
#endif

	va_start(arg, format);
	count = __vfprintf_(&stream, format, arg);
	va_end(arg);

	buffer[count]='\0';

	return count;
}

#endif

/*
+=============================================================================+
| local functions
+=============================================================================+
*/

/*------------------------------------------------------------------------*//**
* \brief Simplified fputc() - adds a character to file-stream
* \details Add one character to file-stream. Use (*put)() function pointer or
* *buffer - whichever is present. Increments the length of stream
*
* \param [in] character is a character which will be appended to stream
* \param [in, out] stream is a pointer to printf_file_t stream
* \return character which was passed as parameter
*//*-------------------------------------------------------------------------*/

static int __fputc_(int character, printf_file_t *stream)
{
	if(character == '\n')
		__fputc_('\r', stream);
	
#if PRINTF_HAVE_PRINTF_SPRINTF == 1
	if (stream->buffer != NULL)					// is buffer pointer valid?
#endif

#if PRINTF_HAVE_SPRINTF == 1
		*stream->buffer++ = (char)character;	// yes - just add the character to it
#endif

#if PRINTF_HAVE_PRINTF_SPRINTF == 1
	else if (stream->put != NULL)				// is put() function pointer valid?
#endif

#if PRINTF_HAVE_PRINTF == 1
		(*stream->put)((char)character);		// yes - use it to add one character to stream
#endif

	stream->length++;							// increment the length of stream

	return character;
}

/*------------------------------------------------------------------------*//**
* \brief Simplified vfprintf() - prints formatted string to stream
* \details Prints a string to stream. Only %s, %d, %x, %c and %%
* specifiers are supported.
*
* \param [in, out] stream is the file-stream which will be used as output
* \param [in] format is a standard format string, additional parameters
* expected - one for each specifier
* \param [in] arg is variable arguments list handle
* \return number of written characters
*//*-------------------------------------------------------------------------*/

static int __vfprintf_(printf_file_t *stream, const char *format, va_list arg)
{
	char character;

	stream->length=0;						// clear the current length of stream

	while ((character = *format++) != '\0')	// loop until termination character '\0'
	{
		if (character != '%')				// specifier found?
			__fputc_(character, stream);	// no - just print the character
		else								// yes
		{
			character=*format++;			// get the character after the specifier

			if (character == '%' || character == 'c')	// %% - print '%' or %c - print single char
			{
				if (character == 'c')		// was that %c?
					character = va_arg(arg, int);	// get the char from va_list
				__fputc_(character, stream);
				continue;
			}

			// %s, %d and %x - these require a string to be copied to stream
			if (character == 's' || character == 'd' || character == 'x')
			{
				char buffer[11];
				char* buffer_ptr;

				if (character == 's')		// %s - get the pointer to string
					buffer_ptr=va_arg(arg, char*);
				else						// %d or %x - convert the number to string
				{
					int base = (character == 'd' ? 10 : 16);

					buffer_ptr = itoa(va_arg(arg, int), buffer, base);
				}

				character = *buffer_ptr;
				while (character)	// copy the string to stream
				{
					__fputc_(character, stream);
					++buffer_ptr;
					character = *buffer_ptr;
				}
				continue;
			}
		}
	}

	return stream->length;
}
