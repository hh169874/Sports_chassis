/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "retarget.h"
#include <arm_math.h>

#include "FreeRTOS.h"
#include "task.h"
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
#define Buzzer_Pin GPIO_PIN_3
#define Buzzer_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_4
#define LED1_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_6
#define LED3_GPIO_Port GPIOE
#define CSS_Pin GPIO_PIN_4
#define CSS_GPIO_Port GPIOA
#define Gyro_Reset_Pin GPIO_PIN_10
#define Gyro_Reset_GPIO_Port GPIOD
#define Start_Button_Pin GPIO_PIN_11
#define Start_Button_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
#define ABS(x)		((x>0)? x: -x)
#define UART_PI &huart2
#define UART_DEBUG &huart1
#define UART_GYRO &huart3
#define DMABuffer_Size 54
#define GYRO_ENABLE //是否使用陀螺仪矫正z轴
//#define VL53L1_ENABLE //是否启用激光测距模块
//#define MPU9250_ENABLE //是否使用mpu9250
//#define HC_SR04_ENABLE //是否使用超声波测距
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
