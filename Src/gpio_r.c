/** \file gpio.c
 * \brief GPIO driver.
 * \details GPIO default configuration and function for configuring a pin
 * \author Freddie Chopin, http://www.freddiechopin.info/
 * \date 2012-03-17
 */

/******************************************************************************
* chip: STM32F4x
* compiler: arm-none-eabi-gcc (GNU Tools for ARM Embedded Processors) 4.6.2
* 	20110921 (release) [ARM/embedded-4_6-branch revision 182083]
*
* prefix: gpio_
*
* available global functions:
* 	void gpio_init(void)
* 	void gpio_pin_cfg(GPIO_TypeDef *port_ptr, uint32_t pin, uint32_t configuration)
*
* available local functions:
*
* available interrupt handlers:
******************************************************************************/

/*
+=============================================================================+
| includes
+=============================================================================+
*/

#include <stdint.h>

#include "gpio_r.h"

/*
+=============================================================================+
| local definitions
+=============================================================================+
*/

#define GPIO_GET_MODER(combination)			(((combination) & 0xF) >> 0)
#define GPIO_GET_OTYPER(combination)		(((combination) & 0xF0) >> 4)
#define GPIO_GET_OSPEEDR(combination)		(((combination) & 0xF00) >> 8)
#define GPIO_GET_PUPDR(combination)			(((combination) & 0xF000) >> 12)
#define GPIO_GET_AFR(combination)			(((combination) & 0xF0000) >> 16)
/*
+=============================================================================+
| module variables
+=============================================================================+
*/

/*
+=============================================================================+
| local functions' declarations
+=============================================================================+
*/

/*
+=============================================================================+
| global functions
+=============================================================================+
*/

/*------------------------------------------------------------------------*//**
* \brief GPIO initialization.
* \details Enables all GPIO ports.
*//*-------------------------------------------------------------------------*/

void gpio_init(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN |
			RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN |
			RCC_AHB1ENR_GPIOFEN | RCC_AHB1ENR_GPIOGEN | RCC_AHB1ENR_GPIOHEN |
			RCC_AHB1ENR_GPIOIEN;			// enable all possible GPIO ports
}

uint8_t ctz(uint32_t n)
{
	uint8_t cnt = 0;
	
	while(!(n&1))
	{
		++cnt;
		n>>=1;
	}
	
	return cnt;
}


/*------------------------------------------------------------------------*//**
* \brief Configures pin.
* \details Configures one pin in one port.
*
* \param [in] port_ptr points to the configuration structure of desired port
* \param [in] pin selects one pin, [0; 15]
* \param [in] configuration is a combined value of MODER, OTYPER, OSPEEDR,
* PUPDR and AFRx register bitfields, allowed values
* {GPIO_IN_FLOATING, GPIO_IN_PULL_UP, GPIO_IN_PULL_DOWN,
* GPIO_OUT_{PP, OD}_{2MHz, 25MHz, 50MHz, 100MHz},
* GPIO_OUT_{PP, OD}_{2MHz, 25MHz, 50MHz, 100MHz}_{PULL_UP, PULL_DOWN},
* GPIO_[AF0; AF15]_{PP, OD}_{2MHz, 25MHz, 50MHz, 100MHz},
* GPIO_[AF0; AF15]_{PP, OD}_{2MHz, 25MHz, 50MHz, 100MHz}_{PULL_UP, PULL_DOWN},
* GPIO_ANALOG}
*//*-------------------------------------------------------------------------*/
void gpio_pin_cfg(GPIO_TypeDef * const __restrict__ port, GpioPin_t pin, GpioMode_t mode)
{
 if (mode & 0x100u) port->OTYPER |= pin;
 else port->OTYPER &= (uint32_t)~pin;
 //pin = __builtin_ctz(pin)*2;
 pin = (GpioPin_t)(ctz(pin) * 2);
 uint32_t reset_mask = ~(0x03u << pin);
 uint32_t reg_val;
 reg_val = port->MODER;
 reg_val &= reset_mask;
 reg_val |= (((mode & 0x600u) >> 9u) << pin );
 port->MODER = reg_val;
 reg_val = port->PUPDR;
 reg_val &= reset_mask;
 reg_val |= (((mode & 0x30u) >> 4u) << pin );
 port->PUPDR = reg_val;
 reg_val = port->OSPEEDR;
 reg_val &= reset_mask;
 reg_val |= (((mode & 0xC0u) >> 6u) << pin);
 port->OSPEEDR = reg_val;
 volatile uint32_t * reg_adr;
 reg_adr = &port->AFR[0];
 pin*=2;
 if ( pin > 28){
	 pin -= 32;
	 reg_adr = &port->AFR[1];
 }
 reg_val = *reg_adr;
 reg_val &= ~(0x0fu << pin);
 reg_val |= (uint32_t)(mode & 0x0ful) << pin;
 *reg_adr = reg_val;
}


/*
+=============================================================================+
| local functions
+=============================================================================+
*/

/*
+=============================================================================+
| ISRs
+=============================================================================+
*/

/******************************************************************************
* END OF FILE
******************************************************************************/
