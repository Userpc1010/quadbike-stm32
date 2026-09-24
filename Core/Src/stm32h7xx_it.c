/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32h7xx_it.h"
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

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi4;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
/* USER CODE BEGIN EV */

extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
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
  * @brief This function handles Pre-fetch fault, memory access fault.
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
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(NRF24L01_INT_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

	// RXNE — пришёл байт
	    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_RXNE)) {
	        uint8_t b = (uint8_t)(UART4->RDR & 0xFF);
	        if (uart4_rx_idx < UART4_RX_BUF_SIZE) {
	            uart4_rx_buf[uart4_rx_active][uart4_rx_idx++] = b;
	            uart4_total_bytes++;
	        }
	    }

	    // IDLE — конец пакета
	    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE)) {
	        __HAL_UART_CLEAR_IDLEFLAG(&huart4);
	        if (uart4_rx_idx > 0) {
	            uint8_t next_head = (uart4_pkt_head + 1) % UART4_PKT_QUEUE_SIZE;
	            if (next_head != uart4_pkt_tail) {
	                // место есть — копируем в очередь
	                for (uint16_t i = 0; i < uart4_rx_idx; i++) {
	                    uart4_pkt_queue[uart4_pkt_head][i] = uart4_rx_buf[uart4_rx_active][i];
	                }
	                uart4_pkt_len_queue[uart4_pkt_head] = uart4_rx_idx;
	                uart4_pkt_head = next_head;
	                uart4_total_packets++;
	            } else {
	                // очередь полна — реальная потеря
	                uart4_pkt_dropped++;
	            }
	            uart4_rx_active ^= 1;
	            uart4_rx_idx = 0;
	        }
	    }

	    // ORE/FE/NE — сброс
	    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_ORE)) {
	        __HAL_UART_CLEAR_OREFLAG(&huart4);
	    }
	    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_FE)) {
	        __HAL_UART_CLEAR_FEFLAG(&huart4);
	    }
	    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_NE)) {
	        __HAL_UART_CLEAR_NEFLAG(&huart4);
	    }

	    // TXE — отдать байт
	    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_TXE) &&
	        __HAL_UART_GET_IT_SOURCE(&huart4, UART_IT_TXE))
	    {
	        if (uart4_tx_tail != uart4_tx_head) {
	            UART4->TDR = uart4_tx_buf[uart4_tx_tail];
	            uart4_tx_tail = (uart4_tx_tail + 1) % UART4_TX_BUF_SIZE;
	        } else {
	            __HAL_UART_DISABLE_IT(&huart4, UART_IT_TXE);
	            __HAL_UART_ENABLE_IT(&huart4, UART_IT_TC);
	        }
	    }

	    // TC — завершение
	    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_TC) &&
	        __HAL_UART_GET_IT_SOURCE(&huart4, UART_IT_TC))
	    {
	        __HAL_UART_CLEAR_FLAG(&huart4, UART_FLAG_TC);
	        __HAL_UART_DISABLE_IT(&huart4, UART_IT_TC);
	        uart4_tx_busy = 0;
	        if (uart4_tx_tail != uart4_tx_head) {
	            uart4_tx_busy = 1;
	            __HAL_UART_ENABLE_IT(&huart4, UART_IT_TXE);
	        }
	    }

  /* USER CODE END UART4_IRQn 0 */
}

/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */

  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */

  /* USER CODE END UART5_IRQn 1 */
}

/**
  * @brief This function handles SPI4 global interrupt.
  */
void SPI4_IRQHandler(void)
{
  /* USER CODE BEGIN SPI4_IRQn 0 */

  /* USER CODE END SPI4_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi4);
  /* USER CODE BEGIN SPI4_IRQn 1 */

  /* USER CODE END SPI4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
