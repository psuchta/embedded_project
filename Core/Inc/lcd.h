/**
 * Copyright (c) 2024
 * \file lcd.h
 * Functional module description: LCD base on HD44780 controller.
 */

#ifndef LCD_H_
#define LCD_H_

#include <stdint.h>
#include "main.h"

// #define LCD_TYPE_1x16
// #define LCD_TYPE_2x8
#define LCD_TYPE_2x16
// #define LCD_TYPE_2x20
// #define LCD_TYPE_2x24
// #define LCD_TYPE_4x16
// #define LCD_TYPE_4x20

#define RS_Pin GPIO_PIN_7
#define RS_GPIO_Port GPIOE
#define EN_Pin GPIO_PIN_11
#define EN_GPIO_Port GPIOE
#define D4_Pin GPIO_PIN_12
#define D4_GPIO_Port GPIOE
#define D5_Pin GPIO_PIN_13
#define D5_GPIO_Port GPIOE
#define D6_Pin GPIO_PIN_14
#define D6_GPIO_Port GPIOE
#define D7_Pin GPIO_PIN_15
#define D7_GPIO_Port GPIOE

/*  Polish chars */
extern const uint8_t Lcd_Pl_A[];
extern const uint8_t Lcd_Pl_C[];
extern const uint8_t Lcd_Pl_E[];
extern const uint8_t Lcd_Pl_L[];
extern const uint8_t Lcd_Pl_N[];
extern const uint8_t Lcd_Pl_O[];
extern const uint8_t Lcd_Pl_S[];
extern const uint8_t Lcd_Pl_Z[];
extern const uint8_t Lcd_Pl_X[];
extern const uint8_t Lcd_Pl_a[];
extern const uint8_t Lcd_Pl_c[];
extern const uint8_t Lcd_Pl_e[];
extern const uint8_t Lcd_Pl_l[];
extern const uint8_t Lcd_Pl_n[];
extern const uint8_t Lcd_Pl_o[];
extern const uint8_t Lcd_Pl_s[];
extern const uint8_t Lcd_Pl_z[];
extern const uint8_t Lcd_Pl_x[];

extern const uint8_t Lcd_DegreeSymbol[];

/**
 * \fn void lcd_execute_handler(void)
 * \brief
 *
 */
void lcd_execute_handler(void);

/**
 * \fn void lcd_int(uint8_t, uint8_t, int)
 * \brief Write a number on the y, x position
 *
 * \param row
 * \param col
 * \param number
 */
void lcd_int(uint8_t row, uint8_t col, int number);

/**
 * \fn void lcd_string(uint8_t, uint8_t, char*)
 * \brief Write a string on the y, x position
 *
 * \param row
 * \param col
 * \param string
 */
void lcd_string(uint8_t row, uint8_t col, char *string);

/**
 * \fn void lcd_char(uint8_t, uint8_t, char)
 * \brief Write a string on the y, x position
 *
 * \param row
 * \param col
 * \param ch
 */
void lcd_char(uint8_t row, uint8_t col, char ch);

/**
 * \fn void lcd_reinit(void)
 * \brief
 *
 */
void lcd_reinit(void);

/**
 * \fn void lcd_clear(void)
 * \brief Clear the screen
 *
 */
void lcd_clear(void);

/**
 * \fn void lcd_createCustomChar(uint8_t, const uint8_t[])
 * \brief Allows us to fill the first 8 CGRAM locations with custom characters
 *
 * \param location 1...8
 * \param charmap
 */
void lcd_createCustomChar(uint8_t location, const uint8_t charmap[]);

/**
 * \fn void led_setBrightness(TIM_HandleTypeDef*, uint32_t, double)
 * \brief Set brightness logarithmic dimming value
 *
 * \param timer - timer handler
 * \param channel - channel of timer to set PWM
 * \param brightness 0...100%
 */
void lcd_ledSetBrightness(TIM_HandleTypeDef *timer, uint32_t channel, double brightness);
#endif
