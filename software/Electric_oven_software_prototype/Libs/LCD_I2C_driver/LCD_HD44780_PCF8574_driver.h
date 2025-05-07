#ifndef LCD_HD_PCF_H
#define LCD_HD_PCF_H

#include "stm32f3xx_hal.h"
#include "stdint.h"

// TODO: add settings
/**
 * @brief Initialise LCD with 4-bit mode
 * @param hi2cHandle pointer to HAL's I2C handle struct
 * @param lcdAddress LCD's I2C address (will be shifted internally)
 */
void lcdInit(I2C_HandleTypeDef* i2cHandle, uint8_t lcdAddress);

#endif
