/**
 * Copyright (c) 2024
 * \file lcd.c
 * Functional module description: LCD base on HD44780 controller.
 * 4 bit interface, no read busy flag, no blocking
 */

// Module Dependency
// ===========================================================================
#include "lcd.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "evTim.h"

// Preprocessor definitions
// ===========================================================================
#if defined(LCD_TYPE_1x16)
#define LCD_ROWS 1
#define LCD_COLS 16
#elif defined(LCD_TYPE_2x8)
#define LCD_ROWS 2
#define LCD_COLS 8
#elif defined(LCD_TYPE_2x16)
#define LCD_ROWS 2
#define LCD_COLS 16
#elif defined(LCD_TYPE_2x20)
#define LCD_ROWS 2
#define LCD_COLS 20
#elif defined(LCD_TYPE_2x24)
#define LCD_ROWS 2
#define LCD_COLS 24
#elif defined(LCD_TYPE_4x16)
#define LCD_ROWS 4
#define LCD_COLS 16
#elif defined(LCD_TYPE_4x20)
#define LCD_ROWS 4
#define LCD_COLS 20
#else
#error "No lcd type"
#endif

#define LCD_NIB         4
#define LCD_BYTE        8
#define LCD_DATA_REG    1
#define LCD_COMMAND_REG 0

/************************************** Command register **************************************/
#define CLEAR_DISPLAY          0x01 // Clears display and returns cursor to the home position (address 0).
#define RETURN_HOME            0x02 // Returns cursor to home position. Also returns display being shifted to the original position.
                                    // DDRAM content remains unchanged.
#define ENTRY_MODE_SET         0x04 // Sets cursor move direction (I/D); specifies to shift the display (S).
                                    // These operations are performed during data read/write.
#define OPT_INC                0x02 // I/D – 0 = decrement cursor position, 1 = increment cursor position;
#define OPT_S                  0x01 // S – 0 = no display shift, 1 = display shift;
#define DISPLAY_ON_OFF_CONTROL 0x08 // Sets on/off of all display (D), cursor on/off (C), and blink of cursor position character (B).
#define OPT_D                  0x04 // D – 0 = display-off, 1 = display on;
#define OPT_C                  0x02 // C – 0 = cursor off, 1 = cursor on;
#define OPT_B                  0x01 // B – 0 = cursor blink off, 1 = cursor blink on ;
#define CURSOR_DISPLAY_SHIFT   0x10 // Sets cursor-move or display-shift (S/C), shift direction (R/L). DDRAM content remains unchanged.
#define OPT_SC                 0x08 // C – 0 = move cursor, 1 = shift display;
#define OPT_RL                 0x04 // R/L – 0 = shift left, 1 = shift right;
#define FUNCTION_SET           0x20 // Sets interface data length (DL), number of display line (N), and character font (F).
#define OPT_DL                 0x10 // DL – 0 = 4-bit interface, 1 = 8-bit interface;
#define OPT_N                  0x08 // N – 0 = 1/8 or 1/11 duty (1 line), 1 = 1/16 duty (2 or 4 lines);
#define OPT_F                  0x04 // F – 0 = 5×8 dots, 1 = 5×10 dots;
#define SET_CGRAM_ADDR         0x40 // Sets the CGRAM address. CGRAM data are sent and received after this setting.
#define SET_DDRAM_ADDR         0x80 // Sets the DDRAM address. DDRAM data are sent and received after this setting.
#define READ_BUSY_FLAG         0x80 // BF – 0 = can accept instruction, 1 = internal operation in progress.

#define Lcd_PortType GPIO_TypeDef*
#define Lcd_PinType uint16_t

// Definitions of variables
// ===========================================================================
///< LCD init commands
static const struct
{
  uint8_t init8[4];
  uint8_t init4[6];
} LcdCmd =
{
  {
    FUNCTION_SET | OPT_DL, // 8-bit mode
    FUNCTION_SET | OPT_DL, // 8-bit mode
    FUNCTION_SET | OPT_DL, // 8-bit mode
    FUNCTION_SET
  },
  {
#if defined(LCD_TYPE_1x16)
    FUNCTION_SET,                   // 4-bit mode
#else
    FUNCTION_SET | OPT_N,           // 4-bit mode
#endif
    DISPLAY_ON_OFF_CONTROL,         // lcdCntr-off, cursor-off, no-blink
    CLEAR_DISPLAY,                  // Clear screen
    ENTRY_MODE_SET | OPT_INC,       // Increment cursor
    DISPLAY_ON_OFF_CONTROL | OPT_D, // lcdCntr-on, cursor-off, no-blink
    RETURN_HOME
  }
};

typedef enum
{
  LCD_RDY,
  LCD_INIT,
  LCD_CMD,
  LCD_DATA
} LcdState_t;

typedef struct
{
  uint8_t screen[LCD_ROWS][LCD_COLS]; // screen content
  struct
  {
    bool init;
    bool update[LCD_ROWS];
  } requestFlags;
  uint8_t lastRow;
  uint8_t commandCounter;
  uint8_t dataCounter;
  evTim_data_t refreshTimer;
  LcdState_t state; // LCD_RDY
  const uint8_t *dataBufferPointer;
  const uint8_t *commandBufferPointer;
  struct
  {
    uint8_t command[8];
    uint8_t pixelMap[8][8];
    bool update[8];
    uint8_t lastNumber;
  } customChar;
} lcdControl_t;

typedef struct
{
  Lcd_PortType *data_port;
  Lcd_PinType *data_pin;
  Lcd_PortType rs_port;
  Lcd_PinType rs_pin;
  Lcd_PortType en_port;
  Lcd_PinType en_pin;
} Lcd_Handle_t;

#if defined(LCD_TYPE_1x16)
static lcdControl_t LcdControl = {
  {"                "}, .requestFlags.init = true
};
#elif defined(LCD_TYPE_2x8)
static lcdControl_t LcdControl = {
  {"        ", "        "}, .requestFlags.init = true
};
#elif defined(LCD_TYPE_2x16)
static lcdControl_t LcdControl = {
  {"                ", "                "}, .requestFlags.init = true
};
#elif defined(LCD_TYPE_2x20)
static lcdControl_t LcdControl = {
  {"                    ", "                    "}, .requestFlags.init = true
};
#elif defined(LCD_TYPE_2x24)
static lcdControl_t LcdControl = {
  {"                        ", "                        "}, .requestFlags.init = true
};
#elif defined(LCD_TYPE_4x16)
static lcdControl_t LcdControl = {
  {"                ", "                ", "                ", "                "}, .requestFlags.init = true
};
#elif defined(LCD_TYPE_4x20)
static lcdControl_t LcdControl = {
  {"                    ", "                    ", "                    ", "                    "}, .requestFlags.init = true
};
#endif

static uint8_t CursorLocateCommands[] =
{
#if defined(LCD_TYPE_4x16)
  SET_DDRAM_ADDR + 0x00,
  SET_DDRAM_ADDR + 0x40,
  SET_DDRAM_ADDR + 0x10,
  SET_DDRAM_ADDR + 0x40
#elif defined(LCD_TYPE_4x20)
  SET_DDRAM_ADDR + 0x00,
  SET_DDRAM_ADDR + 0x40,
  SET_DDRAM_ADDR + 0x14,
  SET_DDRAM_ADDR + 0x54
#else
  SET_DDRAM_ADDR + 0x00,
  SET_DDRAM_ADDR + 0x40
#endif
};

static Lcd_PortType LcdPorts[] = {
  D4_GPIO_Port, D5_GPIO_Port, D6_GPIO_Port, D7_GPIO_Port
};
static Lcd_PinType LcdPins[] = {
  D4_Pin, D5_Pin, D6_Pin, D7_Pin
};

static Lcd_Handle_t LcdPinsStruct =
{
  .data_port = LcdPorts,
  .data_pin = LcdPins,
  .rs_port = RS_GPIO_Port,
  .rs_pin = RS_Pin,
  .en_port = EN_GPIO_Port,
  .en_pin = EN_Pin
};

/*  Polish chars */
const uint8_t Lcd_Pl_A[] = {
  0x0E, 0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x02
};                                                                               // Ą
const uint8_t Lcd_Pl_C[] = {
  0x02, 0x0E, 0x15, 0x10, 0x10, 0x11, 0x0E, 0x00
};                                                                               // Ć
const uint8_t Lcd_Pl_E[] = {
  0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F, 0x02
};                                                                               // Ę
const uint8_t Lcd_Pl_L[] = {
  0x10, 0x12, 0x14, 0x18, 0x10, 0x10, 0x1F, 0x00
};                                                                               // Ł
const uint8_t Lcd_Pl_N[] = {
  0x02, 0x15, 0x19, 0x15, 0x13, 0x11, 0x11, 0x00
};                                                                               // Ń
const uint8_t Lcd_Pl_O[] = {
  0x02, 0x0E, 0x15, 0x11, 0x11, 0x11, 0x0E, 0x00
};                                                                               // Ó
const uint8_t Lcd_Pl_S[] = {
  0x04, 0x0E, 0x10, 0x0E, 0x01, 0x11, 0x0E, 0x00
};                                                                               // Ś
const uint8_t Lcd_Pl_Z[] = {
  0x1F, 0x01, 0x02, 0x1F, 0x08, 0x10, 0x1F, 0x00
};                                                                               // Ż
const uint8_t Lcd_Pl_X[] = {
  0x04, 0x1F, 0x09, 0x02, 0x04, 0x08, 0x1F, 0x00
};                                                                               // Ź
const uint8_t Lcd_Pl_a[] = {
  0x00, 0x00, 0x0E, 0x01, 0x0F, 0x11, 0x0F, 0x04
};                                                                               // ą
const uint8_t Lcd_Pl_c[] = {
  0x02, 0x04, 0x0E, 0x10, 0x10, 0x11, 0x0E, 0x00
};                                                                               // ć
const uint8_t Lcd_Pl_e[] = {
  0x00, 0x00, 0x0E, 0x11, 0x1F, 0x10, 0x0E, 0x04
};                                                                               // ę
const uint8_t Lcd_Pl_l[] = {
  0x0C, 0x04, 0x06, 0x04, 0x0C, 0x04, 0x0E, 0x00
};                                                                               // ł
const uint8_t Lcd_Pl_n[] = {
  0x02, 0x04, 0x1E, 0x11, 0x11, 0x11, 0x11, 0x00
};                                                                               // ń
const uint8_t Lcd_Pl_o[] = {
  0x02, 0x04, 0x0E, 0x11, 0x11, 0x11, 0x0E, 0x00
};                                                                               // ó
const uint8_t Lcd_Pl_s[] = {
  0x02, 0x04, 0x0E, 0x10, 0x0E, 0x01, 0x1E, 0x00
};                                                                               // ś
const uint8_t Lcd_Pl_z[] = {
  0x04, 0x00, 0x1F, 0x02, 0x04, 0x08, 0x1F, 0x00
};                                                                               // ż
const uint8_t Lcd_Pl_x[] = {
  0x02, 0x04, 0x1F, 0x02, 0x04, 0x08, 0x1F, 0x00
};                                                                               // ź

const uint8_t Lcd_DegreeSymbol[] = {
  0x0E, 0x0A, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Prototypes of static functions
// ===========================================================================
/**
 * \fn void lcdWrite(Lcd_HandleTypeDef*, uint8_t, uint8_t)
 * \brief Set len bits on the bus and toggle the enable line
 *
 * \param lcd
 * \param data
 * \param len
 */
static void lcdWrite(Lcd_Handle_t *lcd, uint8_t data, uint8_t len);

/**
 * \fn void lcdWriteData(Lcd_HandleTypeDef*, uint8_t)
 * \brief Write a byte to the data register
 *
 * \param lcd
 * \param data
 */
static void lcdWriteData(Lcd_Handle_t *lcd, uint8_t data);

// Definitions of external functions
// ===========================================================================

void lcd_execute_handler(void)
{
  unsigned int row;
  unsigned int ridx;
  unsigned int charNb;
  uint8_t charNbIdx;

  switch(EvTim_IsReady(&LcdControl.refreshTimer))
  {
    case EVTIM_STOP:
      EvTim_ActivateMs(&LcdControl.refreshTimer, 40);
      break;

    case EVTIM_TIMES_UP:
      EvTim_ActivateUs(&LcdControl.refreshTimer, 625);

      if(LcdControl.state == LCD_RDY)
      {
        if(LcdControl.requestFlags.init)
        {
          LcdControl.requestFlags.init = false;
          LcdControl.commandBufferPointer = LcdCmd.init8;
          LcdControl.commandCounter = sizeof(LcdCmd.init8);
          HAL_GPIO_WritePin(LcdPinsStruct.rs_port, LcdPinsStruct.rs_pin, LCD_COMMAND_REG); // Write to command register
          LcdControl.state = LCD_INIT;
        }
        else
        {
          for(charNb = 1; charNb <= 8; ++charNb)
          {
            charNbIdx = (LcdControl.customChar.lastNumber + charNb) % 8;

            if(LcdControl.customChar.update[charNbIdx])
            {
              LcdControl.customChar.update[charNbIdx] = false;
              LcdControl.commandBufferPointer = &LcdControl.customChar.command[charNbIdx];
              LcdControl.commandCounter = 1;
              LcdControl.dataBufferPointer = LcdControl.customChar.pixelMap[charNbIdx];
              LcdControl.dataCounter = 8;
              LcdControl.state = LCD_CMD;
              LcdControl.customChar.lastNumber = charNbIdx;
              break;
            }
          }

          if(LcdControl.state != LCD_CMD)
          {
            // check for update requests
            for(row = 1; row <= LCD_ROWS; row++)
            {
              ridx = (LcdControl.lastRow + row) % LCD_ROWS;

              if(LcdControl.requestFlags.update[ridx])
              {
                LcdControl.requestFlags.update[ridx] = false; // clear request
                LcdControl.commandBufferPointer = &CursorLocateCommands[ridx]; // set cursor cmd
                LcdControl.commandCounter = 1; // 1 command byte
                LcdControl.dataBufferPointer = LcdControl.screen[ridx];
                LcdControl.dataCounter = LCD_COLS; // no. of data bytes
                LcdControl.lastRow = ridx;
                LcdControl.state = LCD_CMD;
                break;
              }
            }
          }
        }
      }

      switch(LcdControl.state)
      {
        case LCD_INIT: // initial commands in 8-bit mode
          lcdWrite(&LcdPinsStruct, *LcdControl.commandBufferPointer++ >> 4, LCD_NIB); // high nibble only
          EvTim_ActivateUs(&LcdControl.refreshTimer, 4100);

          if(--LcdControl.commandCounter == 0)
          {
            // continue init in 4-bit mode
            LcdControl.commandBufferPointer = LcdCmd.init4;
            LcdControl.commandCounter = sizeof(LcdCmd.init4);
            LcdControl.state = LCD_CMD;
          }

          break;

        case LCD_CMD: // normal commands in 4-bit mode
          if(*LcdControl.commandBufferPointer < 0x04) // clear or home command
          {
            EvTim_ActivateUs(&LcdControl.refreshTimer, 4100);
          }

        case LCD_DATA: // data phase
          lcdWriteData(&LcdPinsStruct, *LcdControl.commandBufferPointer++);

          if(--LcdControl.commandCounter == 0)
          {
            // end of command phase
            if(LcdControl.dataCounter)
            {
              // enter data phase
              LcdControl.commandBufferPointer = LcdControl.dataBufferPointer;
              LcdControl.commandCounter = LcdControl.dataCounter;
              LcdControl.dataCounter = 0;
              HAL_GPIO_WritePin(LcdPinsStruct.rs_port, LcdPinsStruct.rs_pin, LCD_DATA_REG);
              LcdControl.state = LCD_DATA;
            }
            else
            {
              HAL_GPIO_WritePin(LcdPinsStruct.rs_port, LcdPinsStruct.rs_pin,
                                LCD_COMMAND_REG);
              LcdControl.state = LCD_RDY;
            }
          }

          break;

        default:
          ;
      }

      break;

    case EVTIM_IN_PROGRESS:
      break;

    default:
      while(true)
      {
      }
  }
}

void lcd_int(uint8_t row, uint8_t col, int number)
{
  char buffer[11];
  sprintf(buffer, "%d", number);

  lcd_string(row, col, buffer);
}

void lcd_string(uint8_t row, uint8_t col, char *string)
{
  unsigned int i = 0;

  for(; i < strlen(string); i++)
  {
    lcd_char(row, col + i, string[i]);
  }
}

void lcd_char(uint8_t row, uint8_t col, char ch)
{
  if(LCD_ROWS <= row)
  {
    ch = '?';
    row = LCD_ROWS - 1;
  }

  if(LCD_COLS <= col)
  {
    ch = '?';
    col = LCD_COLS - 1;
  }

  if(8 >= ch)
  {
    --ch;
  }

  LcdControl.screen[row][col] = ch;
  LcdControl.requestFlags.update[row] = true;
}

void lcd_reinit(void)
{
  LcdControl.requestFlags.init = true;
}

void lcd_clear(void)
{
  unsigned int i;
  memset(LcdControl.screen, ' ', sizeof(LcdControl.screen));

  for(i = 0; i < LCD_ROWS; i++)
  {
    LcdControl.requestFlags.update[i] = true;
  }
}

void lcd_createCustomChar(uint8_t location, const uint8_t charmap[])
{
  --location;
  location &= 0x7; // we have 8 locations: 0-7
  LcdControl.customChar.command[location] = SET_CGRAM_ADDR | (location << 3);
  memcpy(LcdControl.customChar.pixelMap[location], charmap, 8);
  LcdControl.customChar.update[location] = true;
}

void lcd_ledSetBrightness(TIM_HandleTypeDef *timer, uint32_t channel, double brightness)
{
  uint32_t arr = timer->Instance->ARR;
  brightness /= 100.0;
  uint32_t value = powf(brightness, 2.2) * arr;
  __HAL_TIM_SET_COMPARE(timer, channel, value);
}

// Definitions of static functions
// ===========================================================================

static void lcdWrite(Lcd_Handle_t *lcd, uint8_t data, uint8_t len)
{
  HAL_GPIO_WritePin(lcd->en_port, lcd->en_pin, GPIO_PIN_SET);

  for(uint8_t i = 0; i < len; i++)
  {
    HAL_GPIO_WritePin(lcd->data_port[i], lcd->data_pin[i], (data >> i) & 1);
  }

  HAL_GPIO_WritePin(lcd->en_port, lcd->en_pin, GPIO_PIN_RESET);
}

static void lcdWriteData(Lcd_Handle_t *lcd, uint8_t data)
{
  lcdWrite(lcd, data >> 4, LCD_NIB);
  lcdWrite(lcd, data & 0x0F, LCD_NIB);
}
