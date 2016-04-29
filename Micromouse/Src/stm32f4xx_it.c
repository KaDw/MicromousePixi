/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */
#include "sensor.h"
#include "UI.h"

volatile uint8_t count = 3;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern SPI_HandleTypeDef hspi3;
extern TIM_HandleTypeDef htim6;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line0 interrupt.
*/
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
* @brief This function handles EXTI line2 interrupt.
*/
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
	NVIC_ClearPendingIRQ(EXTI2_IRQn);
	while(1)
	{
		printf_("Jestem w guziku z lewej!\n");
		HAL_Delay(100);
	}
  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
* @brief This function handles EXTI line[15:10] interrupts.
*/
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
 
	NVIC_ClearPendingIRQ(EXTI2_IRQn);
	while(1)
	{
		printf_("Jestem w guziku z prawej!\n");
		HAL_Delay(100);
	}
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
* @brief This function handles SPI3 global interrupt.
*/
void SPI3_IRQHandler(void)
{
  /* USER CODE BEGIN SPI3_IRQn 0 */

  /* USER CODE END SPI3_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi3);
  /* USER CODE BEGIN SPI3_IRQn 1 */

  /* USER CODE END SPI3_IRQn 1 */
}

/**
* @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
*/
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
		switch(count){
		case 1:
			ADC_read_ambient(); // read ambient light
			HAL_GPIO_WritePin(GPIOC, D_LF_Pin, 1); 
			break;
		case 2:
			ADC_read_channel(CH2, &sens[0]); // LF read
			sens[0]-=cal[0];
			HAL_GPIO_WritePin(GPIOC, D_LF_Pin, 0); 
			HAL_GPIO_WritePin(GPIOC, D_RF_Pin, 1); 
			break;
		case 3:
			ADC_read_channel(CH11, &sens[1]); // RF read
			sens[1]-=cal[1];
			HAL_GPIO_WritePin(GPIOC, D_RF_Pin, 0); 
			HAL_GPIO_WritePin(GPIOA, D_L_Pin, 1); 
			HAL_GPIO_WritePin(GPIOC, D_R_Pin, 1); 
			break;
		case 4:
			ADC_read_2channel(CH13, CH12, &sens[2]); // L, R read
			sens[2]-=cal[2];
			sens[3]-=cal[3];
			HAL_GPIO_WritePin(GPIOA, D_L_Pin, 0); 
			HAL_GPIO_WritePin(GPIOC, D_R_Pin, 0);
//			HAL_GPIO_WritePin(GPIOA, D_LS_Pin, 1); 
//			HAL_GPIO_WritePin(GPIOC, D_RS_Pin, 1);
			count = 0;
			break;
		//case 5:
			//ADC_read_2channel(CH3, CH10, ); // LS, RS read
			//HAL_GPIO_WritePin(GPIOA, D_LS_Pin, 0); // side sensors off
			//HAL_GPIO_WritePin(GPIOC, D_RS_Pin, 0);
		//break;
			
	}
			count++;
//	if(count == 1){
//		HAL_GPIO_WritePin(GPIOB, LED1_Pin, 1); // LF_ON
//		cos = 
//	}
//	else if(count == 2){
//		HAL_GPIO_WritePin(GPIOB, LED1_Pin, 0); // LF_OFF
//		HAL_GPIO_WritePin(GPIOB, LED2_Pin, 1); // RF_ON
//	}
//	else if(count == 3){
//		HAL_GPIO_WritePin(GPIOB, LED2_Pin, 0); // RF_OFF
//		HAL_GPIO_WritePin(GPIOB, LED3_Pin, 1); // side sensors on
//		HAL_GPIO_WritePin(GPIOA, LED4_Pin, 1); 
//	}
//	else if(count == 4){
//		HAL_GPIO_WritePin(GPIOB, LED3_Pin, 0); // side sensors off
//		HAL_GPIO_WritePin(GPIOA, LED4_Pin, 0);
//		count = 0;
//	}

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */
  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream0 global interrupt.
*/
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream2 global interrupt.
*/
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
