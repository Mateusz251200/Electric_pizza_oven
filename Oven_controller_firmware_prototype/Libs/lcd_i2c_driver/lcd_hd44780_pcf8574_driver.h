/**
 * @file lcd_hd44780_pcf8574_driver.c
 * @brief Public API for HD44780 LCD driver for STM32, using PCF8574 I2C I/O expander module.
 * @author Mateusz Stelmaszyński
 * @copyright Copyright (c) 2025 Mateusz Stelmaszyński. Licensed under the MIT License. See the LICENSE file in the \software\ directory of this repository for full terms.
 *
 * # Key Features
 * - Asynchronous, DMA-based data transmission (no hardware delays and blocking transmissions, except for the lcdInit function)
 * - Instructions and characters are buffered in a circular queue
 * - The DMA and I2C interface are handled using the STM's HAL
 *
 * # Current limitations
 * - This implementation assumes that no other device besides the LCD is connected to the same I2C interface
 * - No error checking or fault handling
 * - Custom character generation not supported
 * - No busy flag checking (not worth the hassle)
 *
 * # Note
 * Make sure to replace the included stm32f3xx_hal.h file according to the MCU in use
*/

#ifndef LCD_HD_PCF_H
#define LCD_HD_PCF_H

#include "stm32f3xx_hal.h"  // change if using a different MCU
#include "stdbool.h"

/* API functions */

/**
 * @brief Initialise LCD with 4-bit mode and the provided settings
 * @note This function uses three 5 ms delays, and the first 4 transmissions are done in blocking mode
 * @param hi2c pointer to HAL's I2C handle struct
 * @param address LCD's I2C address (will be shifted internally)
 * @param numOfLines refers to cell addressing and display configuration (expected values: 1 or 2) 
 * @param cellHeight number of pixels along a cell's height (8 or 10)
 * @param backlight enables or disables the backlight
 */
void lcdInit(I2C_HandleTypeDef* hi2c, uint8_t address, uint8_t numOfLines, uint8_t cellHeight, bool backlight);

/**
 * @brief Prints a character
 * @param c character
 */
void lcdPrintChar(uint8_t c);

/**
 *@brief Prints a string 
 * @param str pointer to a string
 */
void lcdPrintStr(char* str);

/**
 * @brief Turns the backlight on or off.
 * @note Takes effect after a subsequent data transmission
 * @param state
 */
void lcdBacklightOn(bool state);

/**
 * @brief Turns the backlight on or off immediately. 
 * @param state 
 */
void lcdBacklightOnImmediate(bool state);

/**
 * @brief Clears the LCD
 * @note Simulates a delay by sending dummy bytes
 */
void lcdClear(void);

/**
 * @brief Moves the cursor to the starting position
 * @note Simulates a delay by sending dummy bytes
 */
void lcdReturnHome(void);

/**
 * @brief Sets the printing direction
 */
void lcdPrintLeftToRight(void);

/**
 * @brief Sets the printing direction
 */
void lcdPrintRightToLeft(void);

/**
 * @brief Automatically shift display contents after each character
 * @note The shift direction is the same as the print direction
 * @param state 
 */
void lcdAutoScroll(bool state);

/**
 * @brief Switches the display on or off
 * @param state 
 */
void lcdDisplayOn(bool state);

/**
 * @brief Shows or hides the cursor
 * @param state 
 */
void lcdCursorVisible(bool state);

/**
 * @brief Turns cursor blinking on or off
 * @param state 
 */
void lcdCursorBlink(bool state);

/**
 * @brief Shifts the cursor one place to the right
 */
void lcdShiftCursorRight(void);

/**
 * @brief Shifts the cursor one place to the left
 */
void lcdShiftCursorLeft(void);

/**
 * @brief Shifts display contents one place to the right 
 */
void lcdShiftDisplayRight(void);

/**
 * @brief Shifts display contents one place to the right 
 */
void lcdShiftDisplayLeft(void);

/**
 * @brief Moves the cursor to the desired position
 * @param row number of the desired row, counting from 0
 * @param col number of the desired column, counting from 0
 * @note This function's logic isn't adapted do displays other than 2x16 and 4x20, although it might work somehow
 */
void lcdSetCursorPosition(uint8_t row, uint8_t col);

#endif
