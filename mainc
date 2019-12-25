/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SEG_GPIO_Port GPIOB

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

uint16_t data_adc1_IN0[1];
float print_adc_IN0;
float print_adc_IN0_temp;
float value_adc_IN0;

uint8_t Print_dig1_num;
uint8_t Print_dig2_num;
uint8_t Print_dig3_num;
uint8_t Print_dig4_num;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void SegDisplay(uint16_t Num);
void Seg_print(uint16_t num_count);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ALL_OFF(void)
{
  HAL_GPIO_WritePin(SEG_GPIO_Port, DIG1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SEG_GPIO_Port, DIG2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SEG_GPIO_Port, DIG3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SEG_GPIO_Port, DIG4_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(SEG_GPIO_Port, SEG_A_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SEG_GPIO_Port, SEG_B_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SEG_GPIO_Port, SEG_C_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SEG_GPIO_Port, SEG_D_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SEG_GPIO_Port, SEG_E_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SEG_GPIO_Port, SEG_F_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SEG_GPIO_Port, SEG_G_Pin, GPIO_PIN_RESET);
  //  HAL_GPIO_WritePin(SEG_GPIO_Port, SEG_P_Pin, GPIO_PIN_RESET);
}

void ALL_ON(void)
{
  HAL_GPIO_WritePin(SEG_GPIO_Port, DIG1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SEG_GPIO_Port, DIG2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SEG_GPIO_Port, DIG3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SEG_GPIO_Port, DIG4_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(SEG_GPIO_Port, SEG_A_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SEG_GPIO_Port, SEG_B_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SEG_GPIO_Port, SEG_C_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SEG_GPIO_Port, SEG_D_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SEG_GPIO_Port, SEG_E_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SEG_GPIO_Port, SEG_F_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SEG_GPIO_Port, SEG_G_Pin, GPIO_PIN_SET);
  //  HAL_GPIO_WritePin(SEG_GPIO_Port, SEG_P_Pin, GPIO_PIN_SET);
}

void SegDisplay(uint16_t Num)
{

  switch (Num)
    {
    case 0: // 0
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_B_Pin | SEG_C_Pin | SEG_D_Pin | SEG_E_Pin | SEG_F_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_G_Pin, GPIO_PIN_RESET);
      break;
    case 1: // 1
      HAL_GPIO_WritePin(GPIOB, SEG_B_Pin | SEG_C_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_D_Pin | SEG_E_Pin | SEG_F_Pin | SEG_G_Pin, GPIO_PIN_RESET);
      break;
    case 2: // 2
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_B_Pin | SEG_D_Pin | SEG_E_Pin | SEG_G_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_C_Pin | SEG_F_Pin, GPIO_PIN_RESET);
      break;
    case 3: // 3
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_B_Pin | SEG_C_Pin | SEG_D_Pin | SEG_G_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_E_Pin | SEG_F_Pin, GPIO_PIN_RESET);
      break;
    case 4: // 4
      HAL_GPIO_WritePin(GPIOB, SEG_B_Pin | SEG_C_Pin | SEG_F_Pin | SEG_G_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_D_Pin | SEG_E_Pin, GPIO_PIN_RESET);
      break;
    case 5: // 5
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_C_Pin | SEG_D_Pin | SEG_F_Pin | SEG_G_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_B_Pin | SEG_E_Pin, GPIO_PIN_RESET);
      break;
    case 6: // 6
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_C_Pin | SEG_D_Pin | SEG_E_Pin | SEG_F_Pin | SEG_G_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_B_Pin, GPIO_PIN_RESET);
      break;
    case 7: // 7
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_B_Pin | SEG_C_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_D_Pin | SEG_E_Pin | SEG_F_Pin | SEG_G_Pin, GPIO_PIN_RESET);
      break;
    case 8: // 8
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_B_Pin | SEG_C_Pin | SEG_D_Pin | SEG_E_Pin | SEG_F_Pin | SEG_G_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_P_Pin, GPIO_PIN_RESET);
      break;
    case 9: // 9
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_B_Pin | SEG_C_Pin | SEG_D_Pin | SEG_F_Pin | SEG_G_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_E_Pin, GPIO_PIN_RESET);
      break;
      
    case 10: // A
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_B_Pin | SEG_C_Pin | SEG_E_Pin | SEG_F_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_D_Pin, GPIO_PIN_RESET);
      break;
    case 11: // B
      HAL_GPIO_WritePin(GPIOB, SEG_F_Pin | SEG_E_Pin | SEG_D_Pin | SEG_C_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_B_Pin, GPIO_PIN_RESET);
      break;
    case 12: // C
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_F_Pin | SEG_E_Pin | SEG_D_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_G_Pin | SEG_B_Pin | SEG_C_Pin, GPIO_PIN_RESET);
      break;
    case 13: // D
      HAL_GPIO_WritePin(GPIOB, SEG_C_Pin | SEG_B_Pin | SEG_D_Pin | SEG_E_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_F_Pin, GPIO_PIN_RESET);
      break;
    case 14: // E
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_F_Pin | SEG_D_Pin | SEG_E_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_B_Pin | SEG_C_Pin, GPIO_PIN_RESET);
      break;
    case 15: // F
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_F_Pin | SEG_E_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_D_Pin | SEG_C_Pin | SEG_B_Pin, GPIO_PIN_RESET);
      break;
    case 16:                                             // .
      HAL_GPIO_WritePin(GPIOB, SEG_P_Pin, GPIO_PIN_SET); //�h�b�g
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_B_Pin | SEG_C_Pin | SEG_D_Pin | SEG_E_Pin | SEG_F_Pin, GPIO_PIN_RESET);
      break;
      
    default: // -
      //HAL_GPIO_WritePin(GPIOB, SEG_G_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_B_Pin | SEG_C_Pin | SEG_D_Pin | SEG_E_Pin | SEG_F_Pin | SEG_G_Pin, GPIO_PIN_RESET);
    }
}

void Seg_print(uint16_t num_count)
{
  uint8_t wait_num = 1;
  
  uint8_t Print_dig1_num;
  uint8_t Print_dig2_num;
  uint8_t Print_dig3_num;
  uint8_t Print_dig4_num;

  Print_dig1_num = num_count % 10;
  Print_dig2_num = (num_count / 10) % 10;
  Print_dig3_num = (num_count / 100) % 10;
  Print_dig4_num = (num_count / 1000) % 10;


//  for(volatile uint16_t i = 0; i < 10; i++)
  {

    //Dig1
      HAL_GPIO_WritePin(SEG_GPIO_Port, DIG1_Pin, GPIO_PIN_SET);
      SegDisplay(Print_dig1_num);
      HAL_Delay(wait_num);
      HAL_GPIO_WritePin(SEG_GPIO_Port, DIG1_Pin, GPIO_PIN_RESET);

      //Dig2
      HAL_GPIO_WritePin(SEG_GPIO_Port, DIG2_Pin, GPIO_PIN_SET);
      SegDisplay(Print_dig2_num);
      HAL_Delay(wait_num);
      HAL_GPIO_WritePin(SEG_GPIO_Port, DIG2_Pin, GPIO_PIN_RESET);

      //Dig3
      HAL_GPIO_WritePin(SEG_GPIO_Port, DIG3_Pin, GPIO_PIN_SET);
      SegDisplay(Print_dig3_num);
      HAL_Delay(wait_num);
      HAL_GPIO_WritePin(SEG_GPIO_Port, DIG3_Pin, GPIO_PIN_RESET);

      //Dig4
      HAL_GPIO_WritePin(SEG_GPIO_Port, DIG4_Pin, GPIO_PIN_SET);
      SegDisplay(Print_dig4_num);
      HAL_GPIO_WritePin(SEG_GPIO_Port, SEG_P_Pin, GPIO_PIN_SET);
      HAL_Delay(wait_num);
      HAL_GPIO_WritePin(SEG_GPIO_Port, DIG4_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SEG_GPIO_Port, SEG_P_Pin, GPIO_PIN_RESET);
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  //�N���s�m�F

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)data_adc1_IN0, 2) != HAL_OK)
    {
      Error_Handler();
    }

  if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
    {
      Error_Handler();
    }

  ALL_ON();
  HAL_Delay(2000);
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
  ALL_OFF();

  //////////////////////////////////////////////////////////////////

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {

      value_adc_IN0 = ((float)data_adc1_IN0[0] * 3.39f) / (float)4.096;

      print_adc_IN0      = (0.95f * print_adc_IN0_temp) + ((1.0f - 0.95f) * value_adc_IN0);
      print_adc_IN0_temp = print_adc_IN0;

      Print_dig1_num = (uint16_t)print_adc_IN0 % 10;
      Print_dig2_num = ((uint16_t)print_adc_IN0 / 10) % 10;
      Print_dig3_num = ((uint16_t)print_adc_IN0 / 100) % 10;
      Print_dig4_num = ((uint16_t)print_adc_IN0 / 1000) % 10;

      Seg_print(print_adc_IN0);

      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct   = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct   = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
      Error_Handler();
    }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV6;
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
  hadc1.Instance                   = ADC1;
  hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode    = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion       = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
      Error_Handler();
    }
  /** Configure Regular Channel 
  */
  sConfig.Channel      = ADC_CHANNEL_0;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
      Error_Handler();
    }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
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
  TIM_MasterConfigTypeDef sMasterConfig     = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance               = TIM1;
  htim1.Init.Prescaler         = 7200;
  htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim1.Init.Period            = 1000;
  htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
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
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
      Error_Handler();
    }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SEG_G_Pin | SEG_P_Pin | DIG1_Pin | DIG2_Pin | DIG3_Pin | DIG4_Pin | SEG_A_Pin | SEG_B_Pin | SEG_C_Pin | SEG_D_Pin | SEG_E_Pin | SEG_F_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_R_Pin */
  GPIO_InitStruct.Pin   = LED_R_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_R_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG_G_Pin SEG_P_Pin DIG1_Pin DIG2_Pin 
                           DIG3_Pin DIG4_Pin SEG_A_Pin SEG_B_Pin 
                           SEG_C_Pin SEG_D_Pin SEG_E_Pin SEG_F_Pin */
  GPIO_InitStruct.Pin   = SEG_G_Pin | SEG_P_Pin | DIG1_Pin | DIG2_Pin | DIG3_Pin | DIG4_Pin | SEG_A_Pin | SEG_B_Pin | SEG_C_Pin | SEG_D_Pin | SEG_E_Pin | SEG_F_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
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

  HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
  __nop();
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
