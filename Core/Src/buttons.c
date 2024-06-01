/*
 * buttons.c
 *
 *  Created on: Apr 28, 2024
 *
 */
#include "buttons.h"
#include "string.h"

static int swt = 0;

/*
 * @brief  Check which button was pressed, turn on adequate LED and set index for
 * 			printing on LED
 *
 * @param  mems_info.selected_mems:    pointer to index for printing MEMS data
 * 			0 - accelerometer
 * 			1 - gyroscope
 * 			2 - magnetometer
 *
 * @param  yaw_angle - pointer to the currently measured yaw_angle variable
 * @param  yaw_angle - pointer to the struct which stores currently max observed values
 *
 */

void check_buttons(display_info *mems_info, float *yaw_angle, mems_data_t *mems_max_values)
{
  if(HAL_GPIO_ReadPin(SW0_GPIO_Port, SW0_Pin) == GPIO_PIN_RESET)//SWT1, Right, MAG
  {
    mems_info->show_max_values = false;
    if(1 != swt)
    {
      swt = 1;
      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, RESET);
      HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, RESET);
      HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, RESET);
      HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, SET);

      mems_info->show_angle = false;
      mems_info->selected_mems = 2;
    }
  }
  else if(HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == GPIO_PIN_RESET)//SWT2, center, GYR
  {
    mems_info->show_max_values = false;
    if(2 != swt)
    {
      swt = 2;
      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, RESET);
      HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, SET);
      HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, RESET);
      HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, RESET);

      mems_info->show_angle = false;

      mems_info->selected_mems = 1;
    }
  }
  else if(HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_RESET)//SWT3, Left, ACC
  {
    mems_info->show_max_values = false;
    if(3 != swt)
    {
      swt = 3;
      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, SET);
      HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, RESET);
      HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, RESET);
      HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, RESET);

      mems_info->show_angle = false;
      mems_info->selected_mems = 0;
    }
  }
  else if(HAL_GPIO_ReadPin(SW2_GPIO_Port, SW3_Pin) == GPIO_PIN_RESET)//SWT4, UP, measurement unit change
  {
    swt = 4;

    HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);

    memset(mems_max_values, 0, sizeof(mems_data_t));
    mems_info->display_mode = !(mems_info->display_mode);

    while(HAL_GPIO_ReadPin(SW2_GPIO_Port, SW3_Pin) == GPIO_PIN_RESET){}
  }
  else if(HAL_GPIO_ReadPin(SW2_GPIO_Port, SW4_Pin) == GPIO_PIN_RESET)//SWT5, DOWN, Angle measurement
  {
    swt = 5;
    HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, SET);

    *yaw_angle = 0;
    mems_info->show_angle = true;
    mems_info->show_max_values = false;

    while(HAL_GPIO_ReadPin(SW2_GPIO_Port, SW4_Pin) == GPIO_PIN_RESET){}
  }
  else if(HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) == GPIO_PIN_SET)//USER_BUTTON, max values monitoring
  {
    memset(mems_max_values, 0, sizeof(mems_data_t));
    mems_info->show_angle = false;
    mems_info->show_max_values = true;

    while(HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) == GPIO_PIN_SET){}
  }
}
