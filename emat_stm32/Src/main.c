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
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "src/microrl.h"
#include "event_queue.h"
#include "timer.h"
#include "stm32_microrl_misc.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
static uint32_t timer_counter;
static uint32_t last_offset=13702;
static uint32_t last_width=34;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void MX_TIM1_Init(uint32_t period, uint32_t pulse);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	char str[16] = {0,};
	snprintf (str, 16, "cnt=0x%08X", (unsigned)timer_counter);
	print(str);
}

void EventQueueIsFull(void)
{
	print("Error! EQ is Full!");
	for(;;);
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	eq_queue_element_s ev;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_TIM6_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();

  /* USER CODE BEGIN 2 */
//  MX_TIM1_Init(34034, 34);
  MX_TIM1_Init(last_offset+last_width, last_offset);
//  MX_TIM1_Init(13701, 13702);

	/* Tell system that you will use PD0 for EXTI_Line0 */
  /* Enable and set EXTI Line0 Interrupt to the lowest priority */



  HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  init();
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_OnePulse_Start(&htim1,TIM_CHANNEL_2);

  EQ_RegisterErrorCalback(EventQueueIsFull);
  TIMER_StartAuto(1, 10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  TERM_Task();
	  TIMER_Task();
	  do {
		  EQ_GetEvent(&ev);
		  switch(ev.event){
		  case NO_EVENT:
			  break;
		  case CMD_WIDTH:
			  //TIMER_StartAuto(1, ev.param.uiParam);
			  last_width = ev.param.uiParam*34;
			  TIM1->ARR = last_width+last_offset;
			  break;
		  case CMD_OFFSET:
			  last_offset = ev.param.uiParam*34;
			  TIM1->CCR1 = last_offset;
			  TIM1->ARR = last_width+last_offset;
//			  HAL_TIM_OnePulse_Stop(&htim1,TIM_CHANNEL_2);
//			  sConfig.OCMode = TIM_OCMODE_PWM2;
//			  sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
//			  sConfig.Pulse = ev.param.uiParam*34;
//			  sConfig.ICPolarity = TIM_ICPOLARITY_RISING;
//			  sConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
//			  sConfig.ICFilter = 0;
//			  HAL_TIM_OnePulse_ConfigChannel(&htim1, &sConfig, TIM_CHANNEL_1, TIM_CHANNEL_2);
//			  HAL_TIM_OnePulse_Start(&htim1,TIM_CHANNEL_2);
			  break;
		  case TIMER1_EXPIRED:
			  HAL_GPIO_TogglePin(Interrupt_trigger_pin_GPIO_Port, Interrupt_trigger_pin_Pin);
			  break;
		  case CMD_HV_ON:
			  HAL_GPIO_WritePin(HV_ON_OFF_GPIO_Port, HV_ON_OFF_Pin, GPIO_PIN_SET);
			  break;
		  case CMD_HV_OFF:
			  HAL_GPIO_WritePin(HV_ON_OFF_GPIO_Port, HV_ON_OFF_Pin, GPIO_PIN_RESET);
			  break;
		  }
	  } while (ev.event != NO_EVENT);

//	  HAL_GPIO_WritePin(Interrupt_trigger_pin_GPIO_Port, Interrupt_trigger_pin_Pin, GPIO_PIN_SET);
//	  HAL_Delay(100);
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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

/** NVIC Configuration
*/
void MX_NVIC_Init(void)
{
  /* TIM6_DAC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/* TIM6 init function */
void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 42000;
  HAL_TIM_Base_Init(&htim6);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : Input_interrupt_pin_Pin */
  GPIO_InitStruct.Pin = Input_interrupt_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Input_interrupt_pin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HV_ON_OFF_Pin Interrupt_trigger_pin_Pin */
  GPIO_InitStruct.Pin = HV_ON_OFF_Pin|Interrupt_trigger_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, HV_ON_OFF_Pin|Interrupt_trigger_pin_Pin, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */
/* TIM1 init function */
void MX_TIM1_Init(uint32_t period, uint32_t pulse)
{
  TIM_OnePulse_InitTypeDef sConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 4;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = period;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_OnePulse_Init(&htim1, TIM_OPMODE_SINGLE);
  /*##-2- Configure the Channel 1 in One Pulse mode ##########################*/
  sConfig.OCMode = TIM_OCMODE_PWM2;
  sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfig.Pulse = pulse;
  sConfig.ICPolarity = TIM_ICPOLARITY_RISING;
  sConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfig.ICFilter = 0;

  HAL_TIM_OnePulse_ConfigChannel(&htim1, &sConfig, TIM_CHANNEL_1, TIM_CHANNEL_2);
}

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
