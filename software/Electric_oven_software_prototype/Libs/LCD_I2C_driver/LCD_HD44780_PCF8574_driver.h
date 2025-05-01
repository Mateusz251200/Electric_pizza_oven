#ifndef LCD_HD_PCF_H
#define LCD_HD_PCF_H

#include "stm32f3xx_hal.h"
#include "stdint.h"

/**
 * @brief Initialise LCD with 4-bit mode (TO DO: add settings)
 * @param hi2cHandle pointer to HAL's I2C handle struct
 * @param i2cAddress LCD's I2C address (will be shifted internally)
 */
void LCDinit(I2C_HandleTypeDef* i2cHandle, uint8_t i2cAddress);

#endif
