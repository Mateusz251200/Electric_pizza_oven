add_library(LCD_I2C_driver STATIC
    LCD_HD44780_PCF8574_driver.c
)

# resolve HAL dependency
target_link_libraries(LCD_I2C_driver PUBLIC stm32cubemx)

target_include_directories(LCD_I2C_driver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
