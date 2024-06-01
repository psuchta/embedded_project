/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include "evTim.h"
#include "lcd.h"
#include "imu_lib.h"
#include "buttons.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static evTim_data_t EvTim_myTimer1; /** event timer */
static evTim_data_t EvTim_myTimer2; /** event timer */

static char txt_buffer[BUFF_SIZE]; /** text buffer for printing on LCD */
static mems_t *mems; /** mems_t struct pointer, it allows access to sensors data  */

/** mems_display_info_t struct pointer, it allows to access sensors naming and conversion info  */
static mems_display_info_t *mems_display_info;

/** yaw_angle stores information about current board angle  */
static float yaw_angle = 0;

/** mems_info stores information about currently used display mode  */
static display_info mems_info = { 0, 0, false,false };
/** mems_max_values stores information about largest observed sensor's values */
static mems_data_t mems_max_values = {0.0f, 0.0f, 0.0f};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2); /** Starts the TIM Base generation on second timer */
  lis9ds1_fast_init(); /** prepare LIS9DS1 with simplest settings for continuous data obtaining */
  calibrate_gyroscope();
  lcd_clear(); /** clear LCD if program was previously interrupted */
  memset(txt_buffer, 0x00, BUFF_SIZE * sizeof(char)); /** set clear char buffer */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    lcd_execute_handler(); /* */
    check_buttons(&mems_info, &yaw_angle, &mems_max_values); /** check if SWT3, SWT4 or SWT1 was pushed and set index for printing sensor details */

    //  Read and store sensors data
    if(EvTim_IsReady(&EvTim_myTimer1) != EVTIM_IN_PROGRESS) /** checks the ST timer1 if it reached time event */
    {
      EvTim_ActivateMs(&EvTim_myTimer1, 20); /** generation of time event every 40 miliseconds */
      HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, SET); /** turn on green LED */
      lis9ds1_read_data(); /** read data from mems */

      mems = get_mems_t_struct(); /** returns struct address where sensors data is stored*/
      mems_display_info = get_mems_info_struct();
      // Calculate current angle if flag is true
      if(mems_info.show_angle){
        calculate_yaw_angle(0.02);
      }
      else if(mems_info.show_max_values){
        monitor_max_values();
      }

      HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, RESET); /** turn off green LED */
    }

    //  Display sensors data on the LCD
    if(EvTim_IsReady(&EvTim_myTimer2) != EVTIM_IN_PROGRESS) /** checks the ST timer2 if it reached time event */
    {
      EvTim_ActivateMs(&EvTim_myTimer2, 250);
      if(mems_info.show_angle){
        display_yaw_angle();
      }
      else if(mems_info.show_max_values){
        display_sensor_data(&mems_max_values);
      }
      else{
        display_sensor_data(&mems[mems_info.selected_mems].filtered_data);
      }
    }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void calculate_yaw_angle(float dt) {
  // Read last gyro output from z axis and check if it's not a noise
  if((mems[1].buffer_data[mems[1].buffer_head].z <= -40) || (mems[1].buffer_data[mems[1].buffer_head].z >= 40)){
    yaw_angle += (mems[1].buffer_data[0].z) * dt;
  }
  // If angle reach 360 degrees set angle to zero
  if((int16_t) abs(yaw_angle * GYR_FACTOR) >= 360){
    yaw_angle = 0;
  }
}

void monitor_max_values(void){
  if(abs(mems[mems_info.selected_mems].filtered_data.x) > abs(mems_max_values.x)){
    mems_max_values.x = mems[mems_info.selected_mems].filtered_data.x;
  }
  if(abs(mems[mems_info.selected_mems].filtered_data.y) > abs(mems_max_values.y)){
    mems_max_values.y = mems[mems_info.selected_mems].filtered_data.y;
  }
  if(abs(mems[mems_info.selected_mems].filtered_data.z) > abs(mems_max_values.z)){
    mems_max_values.z = mems[mems_info.selected_mems].filtered_data.z;
  }
}


void display_sensor_data(mems_data_t *mems_values){
  lcd_clear(); /** clear output for LCD (2x16) */
  /** write text and values into char buffer */
  snprintf(
      txt_buffer,
      BUFF_SIZE,
      (mems_info.display_mode == 0) ? "%d%s %s %1.1f" : "%d%s %s %1.2f",
          mems_info.display_mode + 1,
          mems_display_info[mems_info.selected_mems].name,
          mems_display_info[mems_info.selected_mems].data_labels,
         mems_values->x * mems_display_info[mems_info.selected_mems].convert_factors[mems_info.display_mode]
  );
  lcd_string(0, 0, txt_buffer); /** print contents of text buffer into LCD */
  /** write text and values into char buffer */
  snprintf(
      txt_buffer,
      BUFF_SIZE,
      (mems_info.display_mode == 0) ? "%1.1f %1.1f" : "%1.2f %1.2f",
          mems_values->y * mems_display_info[mems_info.selected_mems].convert_factors[mems_info.display_mode],
          mems_values->z * mems_display_info[mems_info.selected_mems].convert_factors[mems_info.display_mode]
  );
  lcd_string(1, 0, txt_buffer); /** print contents of text buffer into LCD */
}


void display_yaw_angle(void){
  lcd_clear(); /** clear output for LCD (2x16) */
  /** write text and values into char buffer */
  snprintf(
      txt_buffer,
      BUFF_SIZE,
      "Angle: %1.1f",
      (float_t) yaw_angle * GYR_FACTOR
  );
  lcd_string(0, 0, txt_buffer); /** print contents of text buffer into LCD */

//  snprintf(
//      txt_buffer,
//      BUFF_SIZE,
//      "Compass: %1.1f",
//      (float_t) calculateAzimuth(&mems[2].buffer_data[mems[2].buffer_head].x, &mems[2].buffer_data[mems[2].buffer_head].y)
//  );
//  lcd_string(1, 0, txt_buffer); /** print contents of text buffer into LCD */
}


// Funkcja obliczająca kąt azymutu
float calculateAzimuth(int16_t *mx, int16_t *my) {
    float azimuth = atan2((float)*mx, (float)*my) * 180 / M_PI;
    if (azimuth < 0) {
        azimuth += 360;
    }
    return azimuth;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET); /** flashing LED 5 */
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
