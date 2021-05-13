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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define LED_R3_Pin GPIO_PIN_2
#define LED_R3_GPIO_Port GPIOE
#define LED_B3_Pin GPIO_PIN_4
#define LED_B3_GPIO_Port GPIOE
#define BUZZER_Pin GPIO_PIN_5
#define BUZZER_GPIO_Port GPIOE
#define BLE_LINKS_WAKE_SW_Pin GPIO_PIN_1
#define BLE_LINKS_WAKE_SW_GPIO_Port GPIOC
#define SD_SCK_Pin GPIO_PIN_5
#define SD_SCK_GPIO_Port GPIOA
#define SD_MISO_Pin GPIO_PIN_6
#define SD_MISO_GPIO_Port GPIOA
#define SD_MOSI_Pin GPIO_PIN_7
#define SD_MOSI_GPIO_Port GPIOA
#define EXT_ANT_Pin GPIO_PIN_9
#define EXT_ANT_GPIO_Port GPIOE
#define DISPLAY_ENABLE_Pin GPIO_PIN_11
#define DISPLAY_ENABLE_GPIO_Port GPIOE
#define BLE_LINKS_WAKE_HW_Pin GPIO_PIN_13
#define BLE_LINKS_WAKE_HW_GPIO_Port GPIOE
#define FET1_CNTRL_Pin GPIO_PIN_15
#define FET1_CNTRL_GPIO_Port GPIOE
#define BLE_LINKS_CMD_MLDP_Pin GPIO_PIN_15
#define BLE_LINKS_CMD_MLDP_GPIO_Port GPIOB
#define LED_B2_Pin GPIO_PIN_10
#define LED_B2_GPIO_Port GPIOD
#define SD_DETECT_Pin GPIO_PIN_13
#define SD_DETECT_GPIO_Port GPIOD
#define LED_G1_Pin GPIO_PIN_7
#define LED_G1_GPIO_Port GPIOC
#define LED_G0_Pin GPIO_PIN_8
#define LED_G0_GPIO_Port GPIOC
#define LED_G3_Pin GPIO_PIN_9
#define LED_G3_GPIO_Port GPIOC
#define LED_R1_Pin GPIO_PIN_15
#define LED_R1_GPIO_Port GPIOA
#define LED_R0_Pin GPIO_PIN_0
#define LED_R0_GPIO_Port GPIOD
#define LED_R2_Pin GPIO_PIN_1
#define LED_R2_GPIO_Port GPIOD
#define BLE_LINKS_ENABLE_Pin GPIO_PIN_3
#define BLE_LINKS_ENABLE_GPIO_Port GPIOD
#define LED_B1_Pin GPIO_PIN_4
#define LED_B1_GPIO_Port GPIOD
#define LED_G2_Pin GPIO_PIN_5
#define LED_G2_GPIO_Port GPIOD
#define SD_CS_Pin GPIO_PIN_6
#define SD_CS_GPIO_Port GPIOD
#define LED_B0_Pin GPIO_PIN_7
#define LED_B0_GPIO_Port GPIOD
#define EEPROM_WP_Pin GPIO_PIN_8
#define EEPROM_WP_GPIO_Port GPIOB
#define EEPROM_CS_Pin GPIO_PIN_9
#define EEPROM_CS_GPIO_Port GPIOB
#define EEPROM_HOLD_Pin GPIO_PIN_0
#define EEPROM_HOLD_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
