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
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdlib.h>
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
/* USER CODE BEGIN Private defines */
#define MY_CAN_ID (15)					// CAN ID OF THIS DEVICE
#define ROUTER_ID (12)					// CAN ID OF ROUTER DEVICE
#define TICKS_PER_CYCLE (200)	// 200
#define STEPPER_PWM_PRESCALER (0)		// clock speed (80000000) divided by (this + 1) = pwm timer speed
#define STEPPER_STEP_DEN (256)
#define GEAR_RATIO (50.0)
#define APB1_TIMER_CLOCK_FREQUENCY (80000000.0)// frequency of the APB1 clock
#define MIN_PWM_TIMER_PERIOD (250)
#define MAX_PWM_TIMER_PERIOD (2000)
#define PD_ANGLE_THRESHOLD (0.05)
#define EPSILON (1) //number in ticks to consider goal reached
#define ENCODER_TO_KALMAN_DEVIATION (0.1) // max angle of deviation to consider something goes wrong and reset the system

//angle limits
#define UPPER_ANGLE_LIMIT (6.28) //hard angle limits (wiring purpose)
#define LOWER_ANGLE_LIMIT (0)

//effort limits
#define UPPER_EFFORT_LIMIT (255) //hard effort limits (heating purpose)
#define LOWER_EFFORT_LIMIT (0)

//velocity limits
#define UPPER_VEL_LIMIT (6.28) //hard vel limits (timing purpose)
#define LOWER_VEL_LIMIT (0)


#define READ 0x01
#define WRITE 0x00

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
