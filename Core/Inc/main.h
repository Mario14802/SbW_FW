/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DRV_nOCTW_Pin GPIO_PIN_13
#define DRV_nOCTW_GPIO_Port GPIOC
#define DRV_nFLT_Pin GPIO_PIN_14
#define DRV_nFLT_GPIO_Port GPIOC
#define DRV_PWRGD_Pin GPIO_PIN_15
#define DRV_PWRGD_GPIO_Port GPIOC
#define ENC_A_Pin GPIO_PIN_0
#define ENC_A_GPIO_Port GPIOA
#define ENC_B_Pin GPIO_PIN_1
#define ENC_B_GPIO_Port GPIOA
#define ENC_Z_Pin GPIO_PIN_2
#define ENC_Z_GPIO_Port GPIOA
#define ADC_IA_Pin GPIO_PIN_3
#define ADC_IA_GPIO_Port GPIOA
#define ADC_IB_Pin GPIO_PIN_4
#define ADC_IB_GPIO_Port GPIOA
#define ADC_SIN_Pin GPIO_PIN_5
#define ADC_SIN_GPIO_Port GPIOA
#define ADC_COS_Pin GPIO_PIN_6
#define ADC_COS_GPIO_Port GPIOA
#define PWM_A_N_Pin GPIO_PIN_7
#define PWM_A_N_GPIO_Port GPIOA
#define HX_CK_Pin GPIO_PIN_2
#define HX_CK_GPIO_Port GPIOB
#define HX_DT_Pin GPIO_PIN_10
#define HX_DT_GPIO_Port GPIOB
#define DRV_DC_CAL_Pin GPIO_PIN_11
#define DRV_DC_CAL_GPIO_Port GPIOB
#define DRV_EN_GATE_Pin GPIO_PIN_12
#define DRV_EN_GATE_GPIO_Port GPIOB
#define LED_2_Pin GPIO_PIN_13
#define LED_2_GPIO_Port GPIOB
#define PWM_B_N_Pin GPIO_PIN_14
#define PWM_B_N_GPIO_Port GPIOB
#define PWM_C_N_Pin GPIO_PIN_15
#define PWM_C_N_GPIO_Port GPIOB
#define PWM_A_P_Pin GPIO_PIN_6
#define PWM_A_P_GPIO_Port GPIOC
#define PWM_B_P_Pin GPIO_PIN_7
#define PWM_B_P_GPIO_Port GPIOC
#define PWM_C_P_Pin GPIO_PIN_8
#define PWM_C_P_GPIO_Port GPIOC
#define PWM_IN_Pin GPIO_PIN_15
#define PWM_IN_GPIO_Port GPIOA
#define LED_1_Pin GPIO_PIN_10
#define LED_1_GPIO_Port GPIOC
#define SPI_SS3_Pin GPIO_PIN_11
#define SPI_SS3_GPIO_Port GPIOC
#define SPI_SS2_Pin GPIO_PIN_12
#define SPI_SS2_GPIO_Port GPIOC
#define SPI_SS1_Pin GPIO_PIN_2
#define SPI_SS1_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
