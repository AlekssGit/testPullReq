/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "test.h"
#include "measurements.h"
#include "state_machine.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint16_t adc_1[4] = {0, };


//uint16_t adc_U_ac_dc[FILTR_NUM] = {0, };
//uint16_t adc_U_AB[FILTR_NUM] = {0, };
//uint16_t adc_I_shunt[FILTR_NUM] = {0, };
//uint16_t adc_temp[FILTR_NUM] = {0, };

//uint16_t adc_1[6] = {0, };

uint32_t U_ac_dc;
uint32_t U_AB;
uint32_t I_shunt;
uint32_t temperature;


uint32_t filtr_U_ac_dc;
uint32_t filtr_U_AB;
uint32_t filtr_I_shunt;
uint32_t filtr_temperature;

short int fl;

uint32_t val;
uint32_t change;

int main_counter;

extern int counter_steps;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(fl == 0)
	{
		fl = 1;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_1, 4);
  HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_4);

  U_AB = 0;
  U_ac_dc = 0;
  temperature = 0;

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_GPIO_WritePin(GPIOB, MR_LCD_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(U_AB_GPIO_Port, U_AB_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(U_ac_dc_GPIO_Port, U_ac_dc_Pin, GPIO_PIN_SET);

  int test_btn = 0;

  val = 0;

  counter_steps = 0;
  fl = 0;

  init_filtr();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch(getState())
	  {
	  case WORK_IBP:
		  if((filtr_U_ac_dc == U_0) | (filtr_U_AB > U_29) | (filtr_U_AB < U_24) | (filtr_I_shunt > I_20) | (filtr_temperature > TEMP_55_ADC))
			  setState(ERROR_IBP);
		  break;
	  case ERROR_IBP:
		  if((filtr_U_ac_dc == U_0) | (filtr_U_AB > U_29) | (filtr_U_AB < U_24) | (filtr_I_shunt > I_20) | (filtr_temperature > TEMP_55_ADC))
		  {
//			HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_SET);
			if(filtr_U_ac_dc == U_0)
			{
				HAL_GPIO_WritePin(U_ac_dc_0_GPIO_Port, U_ac_dc_0_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(U_ac_dc_0_GPIO_Port, U_ac_dc_0_Pin, GPIO_PIN_RESET);
			}

			if(filtr_U_AB > U_29)
			{
				HAL_GPIO_WritePin(U_AB_29_GPIO_Port, U_AB_29_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(U_AB_29_GPIO_Port, U_AB_29_Pin, GPIO_PIN_RESET);
			}

			if(filtr_U_AB < U_24)
			{
				HAL_GPIO_WritePin(U_AB_24_GPIO_Port, U_AB_24_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(U_AB_24_GPIO_Port, U_AB_24_Pin, GPIO_PIN_RESET);
			}

			if(filtr_I_shunt > I_20)
			{
				HAL_GPIO_WritePin(I_mor_20_GPIO_Port, I_mor_20_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(I_mor_20_GPIO_Port, I_mor_20_Pin, GPIO_PIN_RESET);
			}

			if(filtr_temperature > TEMP_55_ADC)
			{
				HAL_GPIO_WritePin(T_high_GPIO_Port, T_high_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(T_high_GPIO_Port, T_high_Pin, GPIO_PIN_RESET);
			}
		  }
		  else
		  {
//			HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_RESET);

			  HAL_GPIO_WritePin(U_AB_29_GPIO_Port, U_AB_29_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(U_AB_24_GPIO_Port, U_AB_24_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(U_ac_dc_0_GPIO_Port, U_ac_dc_0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(T_high_GPIO_Port, T_high_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(I_mor_20_GPIO_Port, I_mor_20_Pin, GPIO_PIN_RESET);

			setState(WORK_IBP);
		  }
		  break;
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //Как только приняли данные по ДМА от АЦП, сразу отфильтруем их и пересчитаем значения
	  if(fl == 1)
	  {
		  mean_vals();
		  measure_data();
		  fl = 0;
	  }

	  //Чтение нажатия кнопки
	  //Если кнопка нажата, то зажигаем все светодиоды и сигналку
	  if(!HAL_GPIO_ReadPin(button_GPIO_Port, button_Pin))
	  {
		  test_btn = 1;

		  all_leds_on();
		  all_segments_on();

		  HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_SET);
	  }
	  else
	  {
		  if(test_btn == 1)
		  {
			  test_btn = 0;

			  all_leds_off();
			  all_segments_off();

			  if(val == 1 )
			  {
				  HAL_GPIO_WritePin(U_AB_GPIO_Port, U_AB_Pin, GPIO_PIN_SET);
				  HAL_Delay(1);
				  HAL_GPIO_WritePin(U_ac_dc_GPIO_Port, U_ac_dc_Pin, GPIO_PIN_RESET);
			  }
			  else
			  {
				  HAL_GPIO_WritePin(U_AB_GPIO_Port, U_AB_Pin, GPIO_PIN_RESET);
				  HAL_Delay(1);
				  HAL_GPIO_WritePin(U_ac_dc_GPIO_Port, U_ac_dc_Pin, GPIO_PIN_SET);
			  }
			  HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_RESET);
		  }
		  if(main_counter % 2  == 0)
		  {
//	  			  measure_data();
		  }
		  if(main_counter % 10  == 0)
		  {
			  change = 1;
			  val = val ^ 1;
			  if(val == 1 )
			  {
				  HAL_GPIO_WritePin(U_AB_GPIO_Port, U_AB_Pin, GPIO_PIN_SET);
  //				  HAL_Delay(1);
				  HAL_GPIO_WritePin(U_ac_dc_GPIO_Port, U_ac_dc_Pin, GPIO_PIN_RESET);
			  }
			  else
			  {
				  HAL_GPIO_WritePin(U_AB_GPIO_Port, U_AB_Pin, GPIO_PIN_RESET);
  //				  HAL_Delay(1);
				  HAL_GPIO_WritePin(U_ac_dc_GPIO_Port, U_ac_dc_Pin, GPIO_PIN_SET);
			  }
		  }
		  update_lcd();
	  }
	  //***************************************************************



  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T4_CC4;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 719;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 50000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 720;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MR_LCD_Pin|dig_3_Pin|dig_2_Pin|dig_1_Pin
                          |I_mor_20_Pin|Opt_Buffer_Pin|Discharge_AB_Pin|Overpower_Pin
                          |NetOn_Pin|buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, U_AB_Pin|U_ac_dc_Pin|U_AB_29_Pin|U_AB_24_Pin
                          |U_ac_dc_0_Pin|T_high_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MR_LCD_Pin dig_3_Pin dig_2_Pin dig_1_Pin
                           I_mor_20_Pin Opt_Buffer_Pin Discharge_AB_Pin Overpower_Pin
                           NetOn_Pin buzzer_Pin */
  GPIO_InitStruct.Pin = MR_LCD_Pin|dig_3_Pin|dig_2_Pin|dig_1_Pin
                          |I_mor_20_Pin|Opt_Buffer_Pin|Discharge_AB_Pin|Overpower_Pin
                          |NetOn_Pin|buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : U_AB_Pin U_ac_dc_Pin U_AB_29_Pin U_AB_24_Pin
                           U_ac_dc_0_Pin T_high_Pin */
  GPIO_InitStruct.Pin = U_AB_Pin|U_ac_dc_Pin|U_AB_29_Pin|U_AB_24_Pin
                          |U_ac_dc_0_Pin|T_high_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : button_Pin */
  GPIO_InitStruct.Pin = button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(button_GPIO_Port, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
