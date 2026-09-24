/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "../../ST7735/st7735.h"
#include "../../nRF24L01p/nrf24l01p.h"
#include "../../pca9685/pca9685.h"
#include "../../DSP/Include/arm_math.h"
#include "protocol.h"
#include "uart4_link.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Button_K1_Pin GPIO_PIN_13
#define Button_K1_GPIO_Port GPIOC
#define NRF24L01_INT_Pin GPIO_PIN_0
#define NRF24L01_INT_GPIO_Port GPIOC
#define NRF24L01_INT_EXTI_IRQn EXTI0_IRQn
#define NRF24L01_MOSI_Pin GPIO_PIN_1
#define NRF24L01_MOSI_GPIO_Port GPIOC
#define NRF24L01_MISO_Pin GPIO_PIN_2
#define NRF24L01_MISO_GPIO_Port GPIOC
#define Stering_Analog_IN_Pin GPIO_PIN_3
#define Stering_Analog_IN_GPIO_Port GPIOC
#define miniPC_UART4_Pin GPIO_PIN_0
#define miniPC_UART4_GPIO_Port GPIOA
#define miniPC_UART4A1_Pin GPIO_PIN_1
#define miniPC_UART4A1_GPIO_Port GPIOA
#define Stering_PWM_Out_Pin GPIO_PIN_2
#define Stering_PWM_Out_GPIO_Port GPIOA
#define Encoder_A_Pin GPIO_PIN_6
#define Encoder_A_GPIO_Port GPIOA
#define Encoder_B_Pin GPIO_PIN_7
#define Encoder_B_GPIO_Port GPIOA
#define LCD_GPIO_LED_Pin GPIO_PIN_10
#define LCD_GPIO_LED_GPIO_Port GPIOE
#define LCD_GPIO_CS_Pin GPIO_PIN_11
#define LCD_GPIO_CS_GPIO_Port GPIOE
#define LCD_SCK_AKA_SCL_Pin GPIO_PIN_12
#define LCD_SCK_AKA_SCL_GPIO_Port GPIOE
#define LCD_GPIO_WR_RS_Pin GPIO_PIN_13
#define LCD_GPIO_WR_RS_GPIO_Port GPIOE
#define LCD_MOSI_AKA_SDA_Pin GPIO_PIN_14
#define LCD_MOSI_AKA_SDA_GPIO_Port GPIOE
#define GNSS_UART5_Pin GPIO_PIN_12
#define GNSS_UART5_GPIO_Port GPIOB
#define GNSS_UART5B13_Pin GPIO_PIN_13
#define GNSS_UART5B13_GPIO_Port GPIOB
#define NRF24L01_CSN_Pin GPIO_PIN_8
#define NRF24L01_CSN_GPIO_Port GPIOD
#define NRF24L01_CE_Pin GPIO_PIN_9
#define NRF24L01_CE_GPIO_Port GPIOD
#define Relay_Move_Back_Pin GPIO_PIN_10
#define Relay_Move_Back_GPIO_Port GPIOD
#define PPS_1Hz_Output_Pin GPIO_PIN_9
#define PPS_1Hz_Output_GPIO_Port GPIOC
#define NRF24L01_SCK_Pin GPIO_PIN_9
#define NRF24L01_SCK_GPIO_Port GPIOA
#define Camera_Source_input_Pin GPIO_PIN_12
#define Camera_Source_input_GPIO_Port GPIOA
#define W25Q64_EEPROM_CE_Pin GPIO_PIN_6
#define W25Q64_EEPROM_CE_GPIO_Port GPIOD
#define W25Q64_EEPROM_Pin GPIO_PIN_7
#define W25Q64_EEPROM_GPIO_Port GPIOD
#define W25Q64_EEPROMB3_Pin GPIO_PIN_3
#define W25Q64_EEPROMB3_GPIO_Port GPIOB
#define W25Q64_EEPROMB4_Pin GPIO_PIN_4
#define W25Q64_EEPROMB4_GPIO_Port GPIOB
#define PCA_9685_Pin GPIO_PIN_6
#define PCA_9685_GPIO_Port GPIOB
#define PCA_9685B7_Pin GPIO_PIN_7
#define PCA_9685B7_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
