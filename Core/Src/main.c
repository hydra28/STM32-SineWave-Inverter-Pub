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
 * i use L = 400 uH with C = 3.3 uF to get ~ 4 kHz Cut-off frequency.*/


#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"
#include "math.h"

#define f_carrier 25000
#define f_fundamental 50 // 50 Hz
#define TIMER_PERIOD 2880
#define ADC_BUF_LEN 3
#define TRIP_LIMIT_PEAK 6 // In Amps
#define TRIP_LIMIT_RMS 6 // In Amps

const float PI = M_PI;

uint16_t adc_buffer[ADC_BUF_LEN];
volatile uint8_t cycle_complete = 0;

int Duty;
int k = 0;
int phase = 0;
int sampleNum;
int TRIP_LIMIT_PEAK_PA0 = (int)(((TRIP_LIMIT_PEAK*0.625/6)+2.52)/3.3f*4095.0f);
int TRIP_LIMIT_RMS_PA0 = (int)(((TRIP_LIMIT_RMS*0.625/6)+2.52)/3.3f*4095.0f);

float radVal;
float sineValue[1000];
float duty_coeff = 0.9f;
float target_peak = 1400.0f; // Target ADC value for PA1
float squared_sum = 0;
float current_rms = 0;
float soft_start_mult = 0.0f;
const float soft_start_step = 0.00002f;
float final_squared_sum = 0;

/* PI Controller Variables */
float Kp = 0.00002f;           // Proportional Gain
float Ki = 0.000001f;           // Integral Gain
float integral_error = 0.0f;   // Accumulated error
float max_integral = 0.2f;     // Anti-windup clamp

void SystemClock_Config(void);

void ISR_SINE(void);

int main(void)
{

  sampleNum = (int)(f_carrier/f_fundamental)/2; // Divided by 2 because only using half cycle

  radVal = 1.0f * PI / (float)sampleNum; // half cycle

  for(int i=1;i<=sampleNum;i++){
	      sineValue[i] = sin(radVal*(i));
  }
  sineValue[sampleNum] = 0.0f;

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUF_LEN);

  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); // IN-RUSH CURRENT RELAY
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // LED FAULT STATUS
  HAL_Delay(2000);


  MX_TIM1_Init();
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);


  while(1)
  {
	  if(cycle_complete) {
		  current_rms = sqrtf(final_squared_sum / (float)sampleNum);
		  cycle_complete = 0;
	  }
  }

}


/*Function for The Interrupt*/
void ISR_SINE(void) {
	k++;

	// Accumulate squares for RMS
	float current_reading = (float)adc_buffer[0];
	squared_sum += (current_reading * current_reading);

	// Soft start
	if (soft_start_mult < 1.0f) {
		soft_start_mult += soft_start_step;
	} else {
		soft_start_mult = 1.0f;

		// 3. PI CONTROLLER (Only runs after soft start finishes)
		// We update once per full cycle at the peak of the negative half-cycle
//		if (k == (sampleNum / 2) && phase == 1) {
//			float actual_peak = (float)adc_buffer[1];
//			float error = target_peak - actual_peak;
//
//			// Update Integral Term
//			integral_error += (error * Ki);
//
//			// Anti-Windup Clamping
//			if (integral_error > max_integral)  integral_error = max_integral;
//			if (integral_error < -max_integral) integral_error = -max_integral;
//
//			// PI Formula: Base Duty (0.8) + Proportional + Integral
//			// Increased Kp/Ki slightly for visible reaction
//			duty_coeff = 0.30f + (error * Kp) + integral_error;
//
//			// Safety Clamps
//			if (duty_coeff > 0.98f) duty_coeff = 0.98f;
//			if (duty_coeff < 0.10f) duty_coeff = 0.10f;
//		}
	}


	Duty = (int)(duty_coeff * TIMER_PERIOD * sineValue[k] * soft_start_mult); // Calculate duty cycle

	// Beginning of Half Cycle
	if(k == 1) {
		// phase = 0 for Positive Half Cycle & phase = 1 for Negative Half Cycle
		if(phase == 0) {
			GPIOA->BRR = GPIO_PIN_9;  // HB OFF
			GPIOB->BSRR = GPIO_PIN_0; // LB ON
			TIM1->CCR1 = Duty;
		} else {
			GPIOB->BRR = GPIO_PIN_0;  // LB OFF
			GPIOA->BSRR = GPIO_PIN_9; // HB ON
			TIM1->CCR1 = TIMER_PERIOD - Duty;
		}
		TIM1->BDTR |= TIM_BDTR_MOE; // Enable outputs now
	}

	// Half Cycle
	if(k > 1 && k <= sampleNum) {
		if(phase == 0) TIM1->CCR1 = Duty;
		else           TIM1->CCR1 = TIMER_PERIOD - Duty;
	}

	// Zero Crossing
	if(k > sampleNum) {
		// Pre-calculate the first Duty of the NEXT phase (k=1 of next phase)
		int nextDuty = (int)(duty_coeff * TIMER_PERIOD * sineValue[1] * soft_start_mult);

		// Prepare CCR1 for the first cycle of the next phase
		if(!phase == 0) TIM1->CCR1 = nextDuty;
		else            TIM1->CCR1 = TIMER_PERIOD - nextDuty;

		k = 0; // Reset counter

		// INSTANTLY stop outputs to prevent the "shadow" glitch
		TIM1->BDTR &= ~(TIM_BDTR_MOE);
		GPIOA->BRR = GPIO_PIN_9; // HB OFF
		GPIOB->BRR = GPIO_PIN_0; // LB OFF
		phase = !phase; // Flip phase for the next interrupt

		// Calculate RMS Current
		final_squared_sum = squared_sum;
		cycle_complete = 1;

		// Reset Sum
		squared_sum = 0;

		// TRIP RMS Current
//		if (current_rms > TRIP_LIMIT_RMS_PA0) {
//				// INSTANTLY disable PWM outputs
//				TIM1->BDTR &= ~(TIM_BDTR_MOE);
//
//				// Force all pins LOW to protect MOSFETs
//				GPIOA->BRR = (GPIO_PIN_8 | GPIO_PIN_7 | GPIO_PIN_9);
//				GPIOB->BRR = (GPIO_PIN_0);
//
//				// LED Indicator
//				GPIOC->BRR = (GPIO_PIN_13);
//
//				Error_Handler();
//				return;
//			}
	}

	// TRIP PEAK Current
//	if (adc_buffer[0] > TRIP_LIMIT_PEAK_PA0) {
//		// INSTANTLY disable PWM outputs
//		TIM1->BDTR &= ~(TIM_BDTR_MOE);
//
//		// Force all pins LOW to protect MOSFETs
//		GPIOA->BRR = (GPIO_PIN_8 | GPIO_PIN_7 | GPIO_PIN_9);
//		GPIOB->BRR = (GPIO_PIN_0);
//
//		// LED Indicator
//		GPIOC->BRR = (GPIO_PIN_13);
//
//		Error_Handler();
//		return;
//	}

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
