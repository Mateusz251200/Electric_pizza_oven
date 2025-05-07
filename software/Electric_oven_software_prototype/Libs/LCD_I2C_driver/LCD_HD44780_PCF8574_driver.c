#include "LCD_HD44780_PCF8574_driver.h"

/* HARDWARE ABSTRACTION */

// interface control bits

#define RS_DATA_REG                 (uint8_t)0x01
#define RS_INSTR_REG                (uint8_t)0
#define EN_HIGH                     (uint8_t)0x04
#define EN_LOW                      (uint8_t)0
#define BL_ON                       (uint8_t)0x08   // backlight
#define BL_OFF                      (uint8_t)0

// instructions and arguments bits

#define CLEAR_DISPLAY_INSTR         (uint8_t)0x01

#define RETURN_HOME_INSTR           (uint8_t)0x02

#define ENTRY_MODE_SET_INSTR_BIT    (uint8_t)0x04
#define EMS_ENTRY_TO_LEFT           (uint8_t)0x02
#define EMS_ENTRY_TO_RIGHT          (uint8_t)0
#define EMS_DISP_SHIFT_ON           (uint8_t)0x01
#define EMS_DISP_SHIFT_OFF          (uint8_t)0

#define DISPLAY_CONTROL_INSTR_BIT   (uint8_t)0x08
#define DC_DISP_ON                  (uint8_t)0x04
#define DC_DISP_OFF                 (uint8_t)0
#define DC_CURSOR_ON                (uint8_t)0x02
#define DC_CURSOR_OFF               (uint8_t)0
#define DC_BLINK_ON                 (uint8_t)0x01
#define DC_BLINK_OFF                (uint8_t)0

#define CURS_DISP_SHIFT_INSTR_BIT   (uint8_t)0x10
#define CDS_SHIFT_DISPLAY           (uint8_t)0x08
#define CDS_SHIFT_CURSOR            (uint8_t)0
#define CDS_SHIFT_RIGHT             (uint8_t)0x04
#define CDS_SHIFT_LEFT              (uint8_t)0

#define FUNCTION_SET_INSTR_BIT      (uint8_t)0x20
#define FS_8BIT_MODE                (uint8_t)0x10
#define FS_4BIT_MODE                (uint8_t)0
#define FS_2LINE_DISP               (uint8_t)0x08
#define FS_1LINE_DISP               (uint8_t)0
#define FS_5X10_DOTS                (uint8_t)0x04
#define FS_5x8_DOTS                 (uint8_t)0

// I2C parameters

I2C_HandleTypeDef* hi2c;
uint8_t address;

// default settings

uint8_t bl = BL_ON;
uint8_t EMS_entryDir = EMS_ENTRY_TO_RIGHT;
uint8_t EMS_dispShift = EMS_DISP_SHIFT_ON;
uint8_t DC_dispState = DC_DISP_ON;
uint8_t DC_cursorVisblty = DC_CURSOR_ON;
uint8_t DC_cursorBlink = DC_BLINK_OFF;
const uint8_t FS_dataLength = FS_4BIT_MODE;
uint8_t FS_linesNum = FS_2LINE_DISP;
uint8_t FS_fontSize = FS_5x8_DOTS;

// instructions to be built from individual bits using the above enum-type variables

uint8_t EMS_instr;
uint8_t DC_instr;
uint8_t FS_instr;


void lcdSendByte(uint8_t rs, uint8_t data) {
    static uint8_t buffer[6] = { 0 };

    // upper half
    buffer[0] = (data & 0xf0) | EN_LOW | rs | bl;
    buffer[1] = buffer[0] | EN_HIGH;
    buffer[2] = buffer[0];
    // lower half
    buffer[3] = (data << 4) | EN_LOW | rs | bl;
    buffer[4] = buffer[3] | EN_HIGH;
    buffer[5] = buffer[3];

    HAL_I2C_Master_Transmit(hi2c, address, buffer, 6, HAL_MAX_DELAY);
}


// data length mode initialisation instructions

#define INIT_8BIT_MODE 0x30
#define INIT_4BIT_MODE 0x20


void lcdInit(I2C_HandleTypeDef* i2cHandle, uint8_t lcdAddress) {
    hi2c = i2cHandle;
    address = lcdAddress << 1;

    /* initialisation sequence:
    * 1. send INIT_8BIT_MODE 3 times to enforce 8-bit data length whatever the previous state was
    * 2. send INIT_4BIT_MODE once to switch to 4-bit data length
    * 3. send display settings (function set, entry mode set, display control)
    * 4. clear display
    */

    uint8_t buffer[3] = { 0 };
    buffer[0] = INIT_8BIT_MODE | EN_LOW | bl;
    buffer[1] = buffer[0] | EN_HIGH;
    buffer[2] = buffer[0];

    for (uint8_t i = 0; i < 3; i++) {
        HAL_I2C_Master_Transmit(hi2c, address, buffer, 3, HAL_MAX_DELAY);
        HAL_Delay(5);
    }

    buffer[0] = INIT_4BIT_MODE | EN_LOW | bl;
    buffer[1] = buffer[0] | EN_HIGH;
    buffer[2] = buffer[0];

    HAL_I2C_Master_Transmit(hi2c, address, buffer, 3, HAL_MAX_DELAY);

    // build instructions from individual bits

    EMS_instr = ENTRY_MODE_SET_INSTR_BIT | EMS_entryDir | EMS_dispShift;
    DC_instr = DISPLAY_CONTROL_INSTR_BIT | DC_dispState | DC_cursorVisblty | DC_cursorBlink;
    FS_instr = FUNCTION_SET_INSTR_BIT | FS_dataLength | FS_linesNum | FS_fontSize;

    lcdSendByte(RS_INSTR_REG, FS_instr);
    lcdSendByte(RS_INSTR_REG, EMS_instr);
    lcdSendByte(RS_INSTR_REG, DC_instr);
    lcdSendByte(RS_INSTR_REG, CLEAR_DISPLAY_INSTR);
}
