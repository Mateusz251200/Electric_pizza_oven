add_library(lcd_i2c_driver STATIC
    lcd_hd44780_pcf8574_driver.c
)

# resolve HAL dependency
target_link_libraries(lcd_i2c_driver PUBLIC stm32cubemx)

target_include_directories(lcd_i2c_driver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
