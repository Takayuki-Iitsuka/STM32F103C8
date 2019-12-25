/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

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

void Digit_ON(uint8_t digit_num)
{
  switch (digit_num)
    {
    case 0:
      break;
    case 1:
      HAL_GPIO_WritePin(SEG_GPIO_Port, DIG1_Pin, GPIO_PIN_SET);
      break;
    case 2:
      HAL_GPIO_WritePin(SEG_GPIO_Port, DIG2_Pin, GPIO_PIN_SET);
      break;
    case 3:
      HAL_GPIO_WritePin(SEG_GPIO_Port, DIG3_Pin, GPIO_PIN_SET);
      break;
    case 4:
      HAL_GPIO_WritePin(SEG_GPIO_Port, DIG4_Pin, GPIO_PIN_SET);
      break;
    default:
      break;
    }
}

void Digit_OFF(uint8_t digit_num)
{
  switch (digit_num)
    {
    case 0:
      break;
    case 1:
      HAL_GPIO_WritePin(SEG_GPIO_Port, DIG1_Pin, GPIO_PIN_RESET);
      break;
    case 2:
      HAL_GPIO_WritePin(SEG_GPIO_Port, DIG2_Pin, GPIO_PIN_RESET);
      break;
    case 3:
      HAL_GPIO_WritePin(SEG_GPIO_Port, DIG3_Pin, GPIO_PIN_RESET);
      break;
    case 4:
      HAL_GPIO_WritePin(SEG_GPIO_Port, DIG4_Pin, GPIO_PIN_RESET);
      break;
    default:
      break;
    }
}

void SegDisplay(uint16_t Num)
{
  switch (Num)
    {
    case 0: // 0
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_B_Pin | SEG_C_Pin | SEG_D_Pin | SEG_E_Pin | SEG_F_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_G_Pin | SEG_P_Pin, GPIO_PIN_RESET);
      break;
    case 1: // 1
      HAL_GPIO_WritePin(GPIOB, SEG_B_Pin | SEG_C_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_D_Pin | SEG_E_Pin | SEG_F_Pin | SEG_G_Pin | SEG_P_Pin, GPIO_PIN_RESET);
      break;
    case 2: // 2
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_B_Pin | SEG_D_Pin | SEG_E_Pin | SEG_G_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_C_Pin | SEG_F_Pin | SEG_P_Pin, GPIO_PIN_RESET);
      break;
    case 3: // 3
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_B_Pin | SEG_C_Pin | SEG_D_Pin | SEG_G_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_E_Pin | SEG_F_Pin | SEG_P_Pin, GPIO_PIN_RESET);
      break;
    case 4: // 4
      HAL_GPIO_WritePin(GPIOB, SEG_B_Pin | SEG_C_Pin | SEG_F_Pin | SEG_G_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_D_Pin | SEG_E_Pin | SEG_P_Pin, GPIO_PIN_RESET);
      break;
    case 5: // 5
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_C_Pin | SEG_D_Pin | SEG_F_Pin | SEG_G_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_B_Pin | SEG_E_Pin | SEG_P_Pin, GPIO_PIN_RESET);
      break;
    case 6: // 6
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_C_Pin | SEG_D_Pin | SEG_E_Pin | SEG_F_Pin | SEG_G_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_B_Pin | SEG_P_Pin, GPIO_PIN_RESET);
      break;
    case 7: // 7
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_B_Pin | SEG_C_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_D_Pin | SEG_E_Pin | SEG_F_Pin | SEG_G_Pin | SEG_P_Pin, GPIO_PIN_RESET);
      break;
    case 8: // 8
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_B_Pin | SEG_C_Pin | SEG_D_Pin | SEG_E_Pin | SEG_F_Pin | SEG_G_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_P_Pin | SEG_P_Pin, GPIO_PIN_RESET);
      break;
    case 9: // 9
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_B_Pin | SEG_C_Pin | SEG_D_Pin | SEG_F_Pin | SEG_G_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_E_Pin | SEG_P_Pin, GPIO_PIN_RESET);
      break;
    case 10: // A
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_B_Pin | SEG_C_Pin | SEG_E_Pin | SEG_F_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_D_Pin | SEG_P_Pin, GPIO_PIN_RESET);
      break;
    case 11: // B
      HAL_GPIO_WritePin(GPIOB, SEG_F_Pin | SEG_E_Pin | SEG_D_Pin | SEG_C_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_B_Pin | SEG_P_Pin, GPIO_PIN_RESET);
      break;
    case 12: // C
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_F_Pin | SEG_E_Pin | SEG_D_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_G_Pin | SEG_B_Pin | SEG_C_Pin | SEG_P_Pin, GPIO_PIN_RESET);
      break;
    case 13: // D
      HAL_GPIO_WritePin(GPIOB, SEG_C_Pin | SEG_B_Pin | SEG_D_Pin | SEG_E_Pin | SEG_G_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_F_Pin | SEG_P_Pin, GPIO_PIN_RESET);
      break;
    case 14: // E
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_D_Pin | SEG_E_Pin | SEG_F_Pin | SEG_G_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_B_Pin | SEG_C_Pin | SEG_P_Pin, GPIO_PIN_RESET);
      break;
    case 15: // F
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_F_Pin | SEG_E_Pin | SEG_G_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_D_Pin | SEG_C_Pin | SEG_B_Pin | SEG_P_Pin, GPIO_PIN_RESET);
      break;
    case 16: // -
      HAL_GPIO_WritePin(GPIOB, SEG_G_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_B_Pin | SEG_C_Pin | SEG_D_Pin | SEG_E_Pin | SEG_F_Pin | SEG_P_Pin, GPIO_PIN_RESET);
      break;
    case 17: // . dot
      HAL_GPIO_WritePin(GPIOB, SEG_P_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_B_Pin | SEG_C_Pin | SEG_D_Pin | SEG_E_Pin | SEG_F_Pin | SEG_G_Pin, GPIO_PIN_RESET);
      break;
    case 18: // all off
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_B_Pin | SEG_C_Pin | SEG_D_Pin | SEG_E_Pin | SEG_F_Pin | SEG_G_Pin | SEG_P_Pin, GPIO_PIN_RESET);
      break;
    default: // error
      HAL_GPIO_WritePin(GPIOB, SEG_B_Pin | SEG_E_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, SEG_A_Pin | SEG_C_Pin | SEG_D_Pin | SEG_F_Pin | SEG_G_Pin | SEG_P_Pin, GPIO_PIN_RESET);
      break;
    }
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
    {
      /* USER CODE BEGIN W1_HardFault_IRQn 0 */
      /* USER CODE END W1_HardFault_IRQn 0 */
    }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
    {
      /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
      /* USER CODE END W1_MemoryManagement_IRQn 0 */
    }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
    {
      /* USER CODE BEGIN W1_BusFault_IRQn 0 */
      /* USER CODE END W1_BusFault_IRQn 0 */
    }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
    {
      /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
      /* USER CODE END W1_UsageFault_IRQn 0 */
    }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break interrupt.
  */
void TIM1_BRK_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_IRQn 0 */

  /* USER CODE END TIM1_BRK_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_BRK_IRQn 1 */

  /* USER CODE END TIM1_BRK_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);

  //  SegDisplay(18);
  //  Digit_OFF(2);

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM1 trigger and commutation interrupts.
  */
void TIM1_TRG_COM_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_IRQn 0 */

  /* USER CODE END TIM1_TRG_COM_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_TRG_COM_IRQn 1 */

  /* USER CODE END TIM1_TRG_COM_IRQn 1 */
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */

  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

/*
  Digit_ON(3);
  SegDisplay(2);
  Digit_OFF(3);

  Digit_ON(4);
  SegDisplay(1);
  Digit_OFF(4);
*/
  //  HAL_Delay(50);
  //    SegDisplay(18);
  //  Digit_OFF(2);

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
