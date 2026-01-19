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
 * PWM HA 		  -> GPIOA 8, PWM LA 		-> GPIOA 7
 * Fundamental HB -> GPIOA 9, Fundamental LB -> GPIOB 0
 */

/* Using Up Count PWM, f = (clock_source/(pre-scaler)/period, Open tim.c to see the detail
 * Duty max @ period, ISR Event TIM1 call @ Top of counter and @ counter reset,
 * ISR called function "ISR_SINE" to calculate the duty cycle for sine generation.
 */

/* For output Low Pass Filter, find cut-off resonance frequency -> f_cut = 1/(2*pi*(LC)^-1)
 * i use L = 3.8 mH with C = 3.3 uF to get ~ 1.5 kHz Cut-off frequency.*/


#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "math.h"

#define f_carrier 60000
#define f_fundamental 60 // 60 Hz
#define TIMER_PERIOD 1199

const float PI = M_PI;

int Duty;
int k = 0;
int phase = 0;
int sampleNum;
float radVal;
float sineValue[1000];
float duty_coeff = 0.9f;
float soft_start_mult = 0.0f;
const float soft_start_step = 0.00002f; // 1.0s / 50000 Hz


void SystemClock_Config(void);

void ISR_SINE(void);

int main(void)
{

  sampleNum = (int)(f_carrier/f_fundamental)/2; // Divided by 2 because only using half cycle

  radVal = 1.0f * PI / sampleNum; // half cycle

  for(int i=1;i<=sampleNum;i++){
	      sineValue[i] = sin(radVal*(i));
  }
  sineValue[sampleNum] = 0.0f;

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_Delay(2000);
  MX_TIM1_Init();
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

  while(1)
  {
	  /* Infinite Loop */
  }

}


/*Function for The Interrupt*/
void ISR_SINE(void) {
	k++;

	// Soft start for 1 second
	if (soft_start_mult < 1.0f) {
		soft_start_mult += soft_start_step;
	}
	else {
		soft_start_mult = 1.0f;
	}

	Duty = (int)(duty_coeff * TIMER_PERIOD * sineValue[k] * soft_start_mult); // Calculate duty cycle

	// Beginning of Half Cycle
	if(k==1) {
		// phase = 0 for Positive Half Cycle & phase = 1 for Negative Half Cycle
		if(phase==0) {
			GPIOA->BRR = GPIO_PIN_9; // Turn OFF pin HB
			GPIOB->BSRR = GPIO_PIN_0; // Turn ON pin LB
			TIM1->CCR1= Duty;
		}

		if(phase==1) {
			GPIOB->BRR = GPIO_PIN_0; // Turn OFF pin LB
			GPIOA->BSRR = GPIO_PIN_9; // Turn ON pin HB
			TIM1->CCR1= TIMER_PERIOD - Duty;
		}
		TIM1->BDTR |= TIM_BDTR_MOE; // Enable  master output to restart PWM
	}

	// Half Cycle
	if(k>1 && k<sampleNum) {
		if(phase==0) TIM1->CCR1= Duty; // Set PWM Duty (HA > LA for Positive Half Cycle)
		if(phase==1) TIM1->CCR1= TIMER_PERIOD - Duty; // Set PWM Duty (LA > HA for Negative Half Cycle)
	}

	// Zero Crossing
	if(k==sampleNum) {
		k = 0;
		TIM1->BDTR &= ~(TIM_BDTR_MOE); // Disable master output to stop PWM when zero crossing
		GPIOA->BRR = GPIO_PIN_9; // Turn OFF pin HB
		GPIOB->BRR = GPIO_PIN_0; // Turn OFF pin LB
		phase = !phase;
	}
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
