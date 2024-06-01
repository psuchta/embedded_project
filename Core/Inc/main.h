/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "mems_types.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#define BUFF_SIZE 25
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef struct {
  int selected_mems;
  int display_mode;
  bool show_max_values;
  bool show_angle;
} display_info;



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
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define USER_BUTTON_Pin GPIO_PIN_0
#define USER_BUTTON_GPIO_Port GPIOA
#define DISP_RS_Pin GPIO_PIN_7
#define DISP_RS_GPIO_Port GPIOE
#define DISP_RW_Pin GPIO_PIN_10
#define DISP_RW_GPIO_Port GPIOE
#define DISP_ENA_Pin GPIO_PIN_11
#define DISP_ENA_GPIO_Port GPIOE
#define DISP_DB4_Pin GPIO_PIN_12
#define DISP_DB4_GPIO_Port GPIOE
#define DISP_DB5_Pin GPIO_PIN_13
#define DISP_DB5_GPIO_Port GPIOE
#define DISP_DB6_Pin GPIO_PIN_14
#define DISP_DB6_GPIO_Port GPIOE
#define DISP_DB7_Pin GPIO_PIN_15
#define DISP_DB7_GPIO_Port GPIOE
#define LD4_Pin GPIO_PIN_12
#define LD4_GPIO_Port GPIOD
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOD
#define LD5_Pin GPIO_PIN_14
#define LD5_GPIO_Port GPIOD
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOD
#define SW3_Pin GPIO_PIN_6
#define SW3_GPIO_Port GPIOC
#define SW4_Pin GPIO_PIN_8
#define SW4_GPIO_Port GPIOC
#define SW2_Pin GPIO_PIN_9
#define SW2_GPIO_Port GPIOC
#define SW1_Pin GPIO_PIN_15
#define SW1_GPIO_Port GPIOA
#define SW0_Pin GPIO_PIN_11
#define SW0_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/**
 * @brief  Calculate current angle of the board
 * @param  dt delta time between gyroscope measurements
 *
 */
void calculate_yaw_angle(float dt);

/**
 * @brief Save the largest observed values ​​in the mems_max_values ​​structure
 *
 */
void monitor_max_values(void);

/**
 * @brief  Show sensor measurements on the LCD display
 * @param  mems_values stores sensor measurements for x,y,z axes
 *
 */
void display_sensor_data(mems_data_t *mems_values);

/**
 * @brief  Show current angle of the board on the LCD display
 *
 */
void display_yaw_angle(void);

float calculateAzimuth(int16_t *mx, int16_t *my);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
