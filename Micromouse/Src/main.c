/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "gpio_r.h"
#include "sensor.h"
#include "fxas21002c.h"
#include "motor.h"
#include "UI.h"
#include "queue.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//char Rx_buf[100];
//char Tx_buf[50];
/* Private variables ---------------------------------------------------------*/
/* ADC */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI3_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM12_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
	MotorInit();
  UI_Init();
	GyroInit();
	GyroCalibrate(0.001, 100);
	HAL_UART_Receive_DMA(&huart1, (uint8_t*)Rx_buf, 1);
	//HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//ENABLE_GYRO();
	//SensorOff();
	ENABLE_ENCODER;
	ENABLE_SENSOR;
	UI_LedOn(UI_LED_GREEN);
	UI_WaitBtnL();
	UI_LedOffAll();
	HAL_Delay(2000);
	q_push(&queue, MotorGoA, 180, 180, 100);
	q_push(&queue, MotorGoA, 200, 200, 100);
//	q_push(&queue, MotorGoA, 200, 200, 100);
//	q_push(&queue, MotorGoA, 200, 200, 100);
	q_push(&queue, MotorTurnA, 90, 0, 100);
//	q_push(&queue, MotorGoA, 100, 100, 100);
//	q_push(&queue, MotorTurnA, 90, 0, 100);
	q_push(&queue, MotorGoA, 200, 200, 100);
	//ADCreadAmbient();
	
	MotorGoA(37, 37, 200);
//	MotorTurnA(180, 0, 250);
//	MotorGoA(180, 180, 200);
//	MotorTurnA(-180, 0, 250);
//	MotorGoA(200, 200, 250);
	HAL_TIM_Base_Start_IT(&htim6);
	while (1)
	{
  /* USER CODE END WHILE */
//		if(distance[5] < 60)
//			MotorGo(500, 500, 200);
//		HAL_Delay(3000);
  /* USER CODE BEGIN 3 */
	}

  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
//void HAL_SYSTICK_Callback(){

//}


// Bluetooth manual mode
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	extern float Kp, Ki, Kd;
	if('a' <= *Rx_buf && *Rx_buf <= 'z')
		*Rx_buf += 'A'-'a';
	
	switch(*Rx_buf){
		case 'W':
		case 'F':
			MotorGoA(300, 300, 200);
		break;
		
		case 'S':
			MotorGoA(-300, -300, 200);
		break;
		
		case 'D':
		//case 'R': 
			MotorTurnA(45, 33, 200);
		break;
			
		case 'A':
		//case 'L':
			MotorTurnA(-45, 33, 200);
		break;
		
		case 'Z':
			MotorStop();
		break;
		
		case 'Q':
			MotorGoA(300, 300, 100);
		break;
		case 'E':
			MotorGoA(300, 300, 300);
		break;
		case 'R':
			MotorGoA(300, 300, 400);
		break;
		
		case 'P':
			Kp += 0.5f;
		break;
		case 'L':
			Kp -= 0.5f;
		
		case 'I':
			Ki += 0.02f;
		break;
		case 'J':
			Ki -= 0.02f;
		break;
		
		case 'Y':
			Kd += 0.01f;
		break;
		case 'G':
			Kd -= 0.01f;
		break;
		}
	
		if(*Rx_buf == 'Y' || *Rx_buf == 'G')
		{
			char * ptr = Tx_buf;
			ptr = strcpy(ptr, "Kp:");
			ptr = _ftoa(Kp, ptr, 2);
			ptr = strcpy(ptr, " Ki: ");
			ptr = _ftoa(Ki, ptr, 2);
			ptr = strcpy(ptr, " Kd: ");
			ptr = _ftoa(Kd, ptr, 2);
			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)Tx_buf, ptr-Tx_buf);
		}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
