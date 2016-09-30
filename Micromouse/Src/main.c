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
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define TXBUFFERSIZE 20

/* Private variables ---------------------------------------------------------*/
/* ADC */
float sum;
uint32_t adc = 4000;
uint8_t Tx_buf = 0x0C;
uint8_t aTxBuffer[] = "abcd\n";
uint8_t Rx_buf = 0xFF; 
uint8_t init_flag = 0;
//uint16_t prevL;
//uint16_t prevR;
//uint16_t currL;
//uint16_t currR;
uint16_t cnt;
extern volatile uint8_t count;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//uint16_t calSensor[4];
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

  /* USER CODE BEGIN 2 */
	MotorInit();
  UI_Init();
	GyroInit();
	GyroCalibrate(0.001, 100);
	HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//MotorSetPWMRaw(50, 50);
//for(int i = 0; i < 5000; ++i)
//{
//	printf_("%d %d\r\n", EncL, EncR);
//	HAL_Delay(1);
//}
//printf("\r\n\nvbat:%dmV\r\nCzekam na start\r\n", UI_BattValue());
//ENABLE_GYRO();
//ENABLE_SENSOR();
SensorOff();
ENABLE_ENCODER();
//MotorRotR90A();
MotorGoA(500, 500, 200);
while(1);
//ENABLE_GYRO();
//UI_WaitBtnL();
//HAL_Delay(600);
//UI_WaitBtnL();
//MotorGoA(500, 500, 50); //mm mm mm/s
//UI_TimerUs(1e6f*MOTOR_DRIVER_T);
//		GyroGetAngle(0.001);
//		adc = TIM7->CNT;
//		printf("MotorUpdate: %dus\r\n", adc-1);
//sensor[0].sens[0] = 5;
//sensor[0].sens[1] = 3;
//sensor[0].sens[2] = 1;
//UI_TimerUs(1e6f*MOTOR_DRIVER_T);
//Sort(&sensor[0]);
//adc = TIM7->CNT;
//printf("Removing pepper noise: %dus\r\n", adc-1);
//HAL_GPIO_WritePin(GPIOB, CS_A_Pin, 1);
init_flag = 1;

extern MotorsV motors; 
  while (1)
  {	
		//GyroGetAngle(0.001);
//		MotorTurn(90, 0, 300);
		//UI_BattControl(); // nie jestem pewny tego sprawdzenia
//		MotorTurnA(1, 1, 1);
		//UI_TimerUs(1e6f*MOTOR_DRIVER_T);
		//UI_TimerUs(1e6*MOTOR_DRIVER_T);
		//GyroGetAngle(MOTOR_DRIVER_T);
		//MotorUpdate();
		//while(UI_TimerBusy())
		//{}
			
		/*if(SENS_RF + SENS_LF < 160) // gdy sciana z przodu
		{
			MotorTurn(-90, 0, 0);
			MotorGoA(300, 300, 400);
		}
		if(SENS_R > 80) // gdy niema sciany z prawej
		{
			UI_DelayUs(60000);
			MotorTurn(90, 0, 0);
			MotorSetPWMRaw(300, 300);
			UI_DelayUs(600);
			MotorGoA(300, 300, 400);
		}*/
//		GyroGetAngle(0.001);
		//HAL_GPIO_WritePin(GPIOB, CS_A_Pin, 0);
		//HAL_GPIO_WritePin(GPIOB, CS_A_Pin, 1);
		//HAL_GPIO_WritePin(GPIOB, CS_A_Pin, 0);
		HAL_UART_Transmit_DMA(&huart1, aTxBuffer, 5);
		//printf("%f\r\n", GyroGetAngle(0.0013));
		//HAL_GPIO_WritePin(GPIOB, CS_A_Pin, 1);
		
		//UI_LedOnAll();
		
		//HAL_Delay(2000);
		//UI_LedOffAll();
//		UI_LedOffAll();
		HAL_Delay(500);
//		adc = TIM7->CNT;
//		printf("MotorUpdate: %dus\r\n", adc-1);
//		if(TIM7->CNT > 30)
//		{
//			MotorStop();
//			while(1)
//			{
//				UI_LedOnAll();
//				UI_DelayUs(25000);
//				UI_LedOffAll();
//				UI_DelayUs(25000);
//			}
//		}
	  //printf("%d\t %d\t %d\t %d\t %d\t %d\t\r\n", SENS_LF, SENS_RF, SENS_L, SENS_R, SENS_RS, SENS_LS);
			
//		//ADCreadChannel(CH9, &adc);
//		printf("%d\t %d\t %d\t %d\t %d\t %d\t %d\r\n", sens[0], sens[1], sens[2], sens[3], sens[4], sens[5], vbat%10 );

  /* USER CODE END WHILE */

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

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
//void HAL_SYSTICK_Callback(){
//	if(init_flag){
//		GyroGetAngle(0.001);
//	}
//}


// Bluetooth manual mode
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//	if('a' <= Rx_buf && Rx_buf <= 'z')
//		Rx_buf += 'A'-'a';
//	
//	switch(Rx_buf){
//		
//		case 'W':
//		case 'F':
//			//MotorSetPWMRaw(600, 600);
//		break;
//		
//		case 'S':
//			//MotorSetPWMRaw(-400, -400);
//		break;
//		
//		case 'D':
//		case 'R': 
//			//MotorSetPWMRaw(600, 300);
//		break;
//			
//		case 'A':
//		case 'L':
//			//MotorSetPWMRaw(300, 600);
//		break;
//		}
//			

//}

/* USER CODE END 4 */

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
