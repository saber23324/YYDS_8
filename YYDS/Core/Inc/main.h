/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <qfsm_user.h>
#include "Mcnamu_wheel_drive.h"
#include "move.h"
#include "sys.h"
#include "mpu.h"
#include "maps.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern unsigned char flag_move;
extern unsigned char flag_site;
extern int head_bias;
#define BEEL_ON		HAL_GPIO_WritePin(BELL_GPIO_Port,BELL_Pin,GPIO_PIN_SET)
#define BEEL_OFF		HAL_GPIO_WritePin(BELL_GPIO_Port,BELL_Pin,GPIO_PIN_RESET)

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HEAD_3_Pin GPIO_PIN_8
#define HEAD_3_GPIO_Port GPIOF
#define HEAD_4_Pin GPIO_PIN_9
#define HEAD_4_GPIO_Port GPIOF
#define HEAD_5_Pin GPIO_PIN_10
#define HEAD_5_GPIO_Port GPIOF
#define BELL_Pin GPIO_PIN_0
#define BELL_GPIO_Port GPIOC
#define DIR_BR_Pin GPIO_PIN_4
#define DIR_BR_GPIO_Port GPIOC
#define DIR_HR_Pin GPIO_PIN_5
#define DIR_HR_GPIO_Port GPIOC
#define DIR_BL_Pin GPIO_PIN_0
#define DIR_BL_GPIO_Port GPIOB
#define DIR_HL_Pin GPIO_PIN_1
#define DIR_HL_GPIO_Port GPIOB
#define HEAD_6_Pin GPIO_PIN_11
#define HEAD_6_GPIO_Port GPIOF
#define LEFT_1_Pin GPIO_PIN_12
#define LEFT_1_GPIO_Port GPIOF
#define LEFT_2_Pin GPIO_PIN_13
#define LEFT_2_GPIO_Port GPIOF
#define LEFT_3_Pin GPIO_PIN_14
#define LEFT_3_GPIO_Port GPIOF
#define LEFT_4_Pin GPIO_PIN_15
#define LEFT_4_GPIO_Port GPIOF
#define LEFT_5_Pin GPIO_PIN_0
#define LEFT_5_GPIO_Port GPIOG
#define LEFT_6_Pin GPIO_PIN_1
#define LEFT_6_GPIO_Port GPIOG
#define piech_pwm_Pin GPIO_PIN_9
#define piech_pwm_GPIO_Port GPIOE
#define yaw_pwm_Pin GPIO_PIN_11
#define yaw_pwm_GPIO_Port GPIOE
#define catch1_Pin GPIO_PIN_13
#define catch1_GPIO_Port GPIOE
#define catch2_Pin GPIO_PIN_14
#define catch2_GPIO_Port GPIOE
#define BACK_1_Pin GPIO_PIN_2
#define BACK_1_GPIO_Port GPIOG
#define BACK_2_Pin GPIO_PIN_3
#define BACK_2_GPIO_Port GPIOG
#define BACK_3_Pin GPIO_PIN_4
#define BACK_3_GPIO_Port GPIOG
#define BACK_4_Pin GPIO_PIN_5
#define BACK_4_GPIO_Port GPIOG
#define BACK_5_Pin GPIO_PIN_6
#define BACK_5_GPIO_Port GPIOG
#define BACK_6_Pin GPIO_PIN_7
#define BACK_6_GPIO_Port GPIOG
#define RIGHT_1_Pin GPIO_PIN_8
#define RIGHT_1_GPIO_Port GPIOG
#define RIGHT_2_Pin GPIO_PIN_10
#define RIGHT_2_GPIO_Port GPIOG
#define RIGHT_3_Pin GPIO_PIN_11
#define RIGHT_3_GPIO_Port GPIOG
#define RIGHT_4_Pin GPIO_PIN_12
#define RIGHT_4_GPIO_Port GPIOG
#define RIGHT_5_Pin GPIO_PIN_13
#define RIGHT_5_GPIO_Port GPIOG
#define RIGHT_6_Pin GPIO_PIN_15
#define RIGHT_6_GPIO_Port GPIOG
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
