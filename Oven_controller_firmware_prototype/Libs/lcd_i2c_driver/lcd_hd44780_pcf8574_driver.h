/**
 * @file lcd_hd44780_pcf8574_driver.h
 * @brief Public API for HD44780 LCD driver for STM32, using PCF8574 I2C I/O expander module. See lcd_hd44780_pcf8574_driver.c for implementation details.
 * @author Mateusz Stelmaszyński
 * @copyright Copyright (c) 2025 Mateusz Stelmaszyński. Licensed under the MIT License. See the LICENSE file in the root directory of this repository for details.
 
# Key Features
- Asynchronous DMA-based data transmission (no blocking transmissions, except for the ones in the lcdInit function)
- LCD instructions are buffered in a circular queue
- DMA and I2C are handled using the STM's HAL

 # Limitations
- If any other device is connected to the same I2C bus as the LCD, the queue must be paused before transmitting or receiving data from that device, and resumed after completion (relevant functions are provided)
- No built-in conversion of variables to ASCII strings
- No custom character generation (yet)
- No busy flag checking

# Requirements:
- Replace the included stm32f3xx_hal.h file according to your MCU
- Enable I2C and DMA interrupts
- In your main program file, implement HAL_I2C_MasterTxCpltCallback and HAL_I2C_ErrorCallback according to these minimal examples:

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c) {
    if (hi2c == &hi2c1)
        lcdFlushQueue();
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c) {
    if (hi2c == &hi2c1)
        lcdi2cErrorHandler();
}
*/

#ifndef LCD_HD_PCF_H
#define LCD_HD_PCF_H

#include "stm32f3xx_hal.h"  // change if using a different MCU
#include "stdbool.h"

/* Status info */

typedef enum LCDStatus_t {
    LCD_OK,
    LCD_QUEUE_PAUSED_AND_FULL,  // All new entries are discarded unless the queue is explicitly and successfully resumed.
    LCD_QUEUE_FULL_TIMEOUT,     // Unable to transmit and dequeue the oldest entry within 10ms of waiting. This results in discarding the new pending entry and pausing the queue to prevent further blocking loops.
    LCD_QUEUE_EMPTY,
    LCD_I2C_TX_INIT_FAIL,       // Failed to initialise I2C DMA transmission.
    LCD_I2C_ERROR,              // There is a persisting I2C error. This results in pausing the queue.
} LCDStatus_t;

/* API functions */

/**
 * @brief Initialise LCD to 4-bit mode with the provided settings
 * @note This function uses 3 5ms delays and 4 blocking transmissions
 * @param hi2c pointer to HAL's I2C handle struct
 * @param address LCD's I2C address (will be shifted internally)
 * @param numOfLines refers to cell addressing and display configuration (expected values: 1 or 2) 
 * @param cellHeight number of pixels along a cell's height (8 or 10)
 * @param backlight enables or disables the backlight
 */
LCDStatus_t lcdInit(I2C_HandleTypeDef* hi2c, uint8_t address, uint8_t numOfLines, uint8_t cellHeight, bool backlight);

/**
 * @brief Prints a character
 * @param c character
 */
LCDStatus_t lcdPrintChar(uint8_t c);

/**
 *@brief Prints a string 
 * @param str pointer to a string
 */
LCDStatus_t lcdPrintStr(char* str);

/**
 * @brief Sets the backlight on or off in a subsequent data transmission (avoids cluttering the queue).
 * @param state
 */
void lcdSetBacklight(bool state);

/**
 * @brief Sets the backlight on or off (immediately puts the instruction in the queue). 
 * @param state 
 */
LCDStatus_t lcdSetBacklightNow(bool state);

/**
 * @brief Clears the LCD
 * @note Simulates a delay on the I2C bus by sending dummy bytes
 */
LCDStatus_t lcdClear(void);

/**
 * @brief Moves the cursor to the starting position
 * @note Simulates a delay on the I2C bus by sending dummy bytes. If you only care about moving the cursor to row 0 and column 0, consider using lcdSetCursorPos(0, 0) - it's faster (doesn't require dummy bytes).
 */
LCDStatus_t lcdReturnHome(void);

/**
 * @brief Sets the printing direction - left to right
 */
LCDStatus_t lcdSetLTR(void);

/**
 * @brief Sets the printing direction - right to left
 */
LCDStatus_t lcdSetRTL(void);

/**
 * @brief Automatically shift display contents after each character
 * @note The shift direction is the same as the print direction
 * @param state 
 */
LCDStatus_t lcdSetAutoScroll(bool state);

/**
 * @brief Switches the display on or off
 * @param state 
 */
LCDStatus_t lcdSetDisplay(bool state);

/**
 * @brief Shows or hides the cursor
 * @param state 
 */
LCDStatus_t lcdSetCursorVisible(bool state);

/**
 * @brief Sets cursor blinking on or off
 * @param state 
 */
LCDStatus_t lcdSetCursorBlink(bool state);

/**
 * @brief Shifts the cursor one place to the right
 */
LCDStatus_t lcdShiftCursorR(void);

/**
 * @brief Shifts the cursor one place to the left
 */
LCDStatus_t lcdShiftCursorL(void);

/**
 * @brief Shifts display contents one place to the right 
 */
LCDStatus_t lcdShiftDisplayR(void);

/**
 * @brief Shifts display contents one place to the right 
 */
LCDStatus_t lcdShiftDisplayL(void);

/**
 * @brief Moves the cursor to the desired position
 * @param row number of the desired row, counting from 0
 * @param col number of the desired column, counting from 0
 * @note This function's logic is adapted to 2x16 and 4x20 displays. It should work for other formats, but the row and column numbers might be off.
 */
LCDStatus_t lcdSetCursorPos(uint8_t row, uint8_t col);

/**
 * @brief Function to be called inside HAL_I2C_MasterTxCpltCallback
 */
void lcdFlushQueue(void);

/**
 * @brief Function to be called inside HAL_I2C_ErrorCallback. It prevents queue stalling if a transmission fails and HAL_I2C_MasterTxCpltCallback doesn't get called.
 */
void lcdi2cErrorHandler(void);

/* queue control and status */

/**
 * @brief Pauses the queue
 * @note The I2C bus becomes free to use after a potential ongoing transmission is completed.
 */
void lcdQueuePause(void);

/**
 * @brief Checks whether the queue is paused and no entry is being transmitted
 * @return bool
 */
bool lcdQueueIsPaused(void);

/**
 * @brief Resumes the queue and begins transmitting entries
 * @return LCDStatus_t 
 */
LCDStatus_t lcdQueueResume(void);

/**
 * @brief Checks whether the queue is full
 * @return bool
 */
bool lcdQueueIsFull(void);

/**
 * @brief Returns the LCD driver's status
 * @return LCDStatus_t 
 */
LCDStatus_t lcdGetStatus(void);

#endif
