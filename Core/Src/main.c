/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Uni-polar Sine-wave Inverter
 * PWM HA 		  -> GPIO PIN 8, PWM LA 		-> GPIO PIN 7
 * Fundamental HB -> GPIO PIN 9, Fundamental LB -> GPIO PIN 10
 */

/* Using Complimentary Center-Aligned PWM, f = (clock_source/(pre-scaler)/period, Open tim.c to see the detail
 * Duty max @ period, ISR Event TIM1 call @ Top of counter and @ counter reset (twice per cycle),
 * ISR called function "ISR_SINE" to calculate the duty cycle for sine generation.
 */

/* For output Low Pass Filter, find cut-off resonance frequency -> f_cut = 1/(2*pi*(LC)^-1)
 * i use L = 660 uH with C = 47/2 uF to get ~ 600 Hz Cut-off frequency.*/


#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "math.h"

#define f_carrier 25000
#define f_fundamental_1 50
#define f_fundamental_2 67.5
#define TIMER_PERIOD 1439 //((72 MHz/25 KHz)/2)-1

const float PI = M_PI;
const float duty_coeff_1 = 0.741;
const float duty_coeff_2 = 1.0;
int Duty;
int k = 0;
int b = 0;
int f_fundamental;
int sampleNum;
float radVal;
float sineValue[1000];
float duty_coeff = duty_coeff_2;


void SystemClock_Config(void);

void ISR_SINE(void);


int main(void)
{

  sampleNum = (int)(f_carrier/f_fundamental_1);

  radVal = 2 * PI / sampleNum;

  for(int i=1;i<sampleNum+1;i++){
	      sineValue[i] = sin(radVal*(i));
  }

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();

  MX_TIM1_Init();
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

  while (1)
  {
	  /* Infinite Loop */
  }

}


/*Function for The Interrupt*/
void ISR_SINE(void) {
	if(b>1){
	  Duty = (int)(duty_coeff*TIMER_PERIOD*sineValue[k]);
	  if (k==0) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
		TIM1->CCR1= 0;
	  }
	  if (k==(sampleNum/2)) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
		TIM1->CCR1 = TIMER_PERIOD;
	  }
	  if (k>0 && k<(sampleNum/2))  {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		TIM1->CCR1 = Duty;
	  }
	  if (k>0 && k>(sampleNum/2) ) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
		TIM1->CCR1 = TIMER_PERIOD + Duty;
	  }
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  k++;
	  if (k > sampleNum ) k=0;
	  b=0;
	}
	b++;
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
