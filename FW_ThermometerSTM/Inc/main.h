/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

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
#define PMODE_Pin GPIO_PIN_15
#define PMODE_GPIO_Port GPIOC
#define VCC_DS_Pin GPIO_PIN_0
#define VCC_DS_GPIO_Port GPIOA
#define DS_DATA_Pin GPIO_PIN_1
#define DS_DATA_GPIO_Port GPIOA
#define CE_Pin GPIO_PIN_2
#define CE_GPIO_Port GPIOA
#define nCS_Pin GPIO_PIN_3
#define nCS_GPIO_Port GPIOA
#define IRQ_Pin GPIO_PIN_4
#define IRQ_GPIO_Port GPIOA
#define VCC_SHT_Pin GPIO_PIN_1
#define VCC_SHT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
//SENSOR ID: 
//	bits 7..3: id
//  bits 2..0: type
#define SENS_ID	0x03

#define STYPE_Tds18b20		0x01
#define STYPE_Tsht21			0x02
#define STYPE_Hsht21			0x03
#define STYPE_Tbmp280			0x05
#define STYPE_Pbmp280			0x06

#define SENSOR_ID(type)	(SENS_ID<<3 | (0x07 & type) )


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
