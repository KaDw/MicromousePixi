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
#include "motor.h"

volatile uint16_t count = 0;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_spi3_rx;
extern DMA_HandleTypeDef hdma_spi3_tx;
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
* @brief This function handles EXTI line2 interrupt.
*/
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
	NVIC_ClearPendingIRQ(EXTI2_IRQn);
//	while(1)
//	{
//		printf_("Jestem w guziku z lewej!\n");
//		HAL_Delay(100);
//	}
  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
* @brief This function handles DMA1 stream0 global interrupt.
*/
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi3_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
* @brief This function handles DMA1 stream5 global interrupt.
*/
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi3_tx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
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
 
	// PRAWY PRZYCISK
	//wywalic pozniej
	MotorStop();
	HAL_TIM_Base_Stop_IT(&htim6);
	
	
	count = 0;
	uint32_t bat;
	uint32_t temp;
	extern ADC_HandleTypeDef hadc1;
	
	NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
	EXTI->PR |= EXTI_PR_PR15; // pending register 15
	
	// bat
	ADC1->SQR1 &= ~ADC_SQR1_L; // 1 conversion
	ADC1->SQR3 = ADC_CHANNEL_9; // channel to be converted
	HAL_ADC_Start(&hadc1);
	bat = HAL_ADC_GetValue(&hadc1);
	
	//todo: tutaj zczytuje 2 razy to samo
	//temp
	ADC1->SQR3 = ADC_CHANNEL_9; // channel to be converted
	HAL_ADC_Start(&hadc1);
	temp = HAL_ADC_GetValue(&hadc1);
	
	printf_("bat:%d  temp:%d\n", bat, temp);
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
* @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
*/
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
		switch(count){
		case 1: 
			ADCreadAmbient(); // read ambient light
			HAL_GPIO_WritePin(GPIOC, D_LF_Pin, GPIO_PIN_SET); 
			break;
		case 3: // 40us
			ADCreadChannel(CH2, &read[0]); // LF read
			HAL_GPIO_WritePin(GPIOC, D_LF_Pin, GPIO_PIN_RESET); 
			//HAL_GPIO_WritePin(GPIOC, D_RF_Pin, GPIO_PIN_SET); 
			break;
		case 6: // 100us
			HAL_GPIO_WritePin(GPIOC, D_RF_Pin, GPIO_PIN_SET);	
			//read[0] = ((read[0] - cal[0]) + 7*fuzzy[0])/8;
			sens[0] = read[0];
			//sens[0] = LinADC(&read[0]);
			//fuzzy[0] = read[0];
			//ADCreadChannel(CH11, &sens[1]); // RF read
			//sens[1]-=cal[1];
			//HAL_GPIO_WritePin(GPIOC, D_RF_Pin, GPIO_PIN_RESET); 
			//HAL_GPIO_WritePin(GPIOA, D_L_Pin, GPIO_PIN_SET); 
			//HAL_GPIO_WritePin(GPIOC, D_R_Pin, GPIO_PIN_SET); 
			break;
		case 8: // 140us
			ADCreadChannel(CH11, &read[1]); // RF read
			HAL_GPIO_WritePin(GPIOC, D_RF_Pin, GPIO_PIN_RESET);
//			ADCread2Channel(CH13, CH12, &sens[2]); // L, R read
//			sens[2]-=cal[2];
//			sens[3]-=cal[3];
//			HAL_GPIO_WritePin(GPIOA, D_L_Pin, GPIO_PIN_RESET); 
//			HAL_GPIO_WritePin(GPIOC, D_R_Pin, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(GPIOA, D_LS_Pin, GPIO_PIN_SET); 
//			HAL_GPIO_WritePin(GPIOC, D_RS_Pin, GPIO_PIN_SET);
			break;
		case 11: // 200us
			HAL_GPIO_WritePin(GPIOA, D_L_Pin, GPIO_PIN_SET); 
			HAL_GPIO_WritePin(GPIOC, D_R_Pin, GPIO_PIN_SET); 
			read[1] = ((read[1] - cal[1]) + 7*fuzzy[1])/8;
			sens[3] = read[1];
			sens[1] = LinADC(&read[1]);
			fuzzy[1] = read[1];
			break;
		case 13: // 240us
			ADCread2Channel(CH13, CH12, &read[0]); // L, R read
			read[0]-=cal[2];
			read[1]-=cal[3];
			//sens[2] = LinADC(&read[0]);
			//sens[3] = LinADC(&read[1]);
			HAL_GPIO_WritePin(GPIOA, D_L_Pin, GPIO_PIN_RESET); 
			HAL_GPIO_WritePin(GPIOC, D_R_Pin, GPIO_PIN_RESET); 
			break;
		case 16: // 300us
			HAL_GPIO_WritePin(GPIOA, D_LS_Pin, GPIO_PIN_SET); 
			HAL_GPIO_WritePin(GPIOC, D_RS_Pin, GPIO_PIN_SET);
			break;
		case 18: // 340us
			ADCread2Channel(CH3, CH10, &read[0]); // LS, RS read
			read[0]-=cal[4];
			read[1]-=cal[5];
			sens[4] = LinADC(&read[0]);
			sens[5] = LinADC(&read[1]);
			HAL_GPIO_WritePin(GPIOA, D_LS_Pin, GPIO_PIN_RESET); // side sensors off
			HAL_GPIO_WritePin(GPIOC, D_RS_Pin, GPIO_PIN_RESET);
			count = 0; // trzeba to przsuwac do ostatniego case
			break;
	}
		
		++count;
		
		if(count >= 50)
			count = 0;
		
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
