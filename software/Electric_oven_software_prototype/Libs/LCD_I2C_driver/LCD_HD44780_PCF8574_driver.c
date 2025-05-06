#include "LCD_HD44780_PCF8574_driver.h"

// hardware interface control bits

#define RS_BIT  (uint8_t)0x01
#define RW_BIT  (uint8_t)0x02
#define EN_BIT  (uint8_t)0x04
#define BL_BIT  (uint8_t)0x08 // backlight

// instructions and arguments bits

#define CLEAR_DISPLAY_INSTR         (uint8_t)0x01

#define RETURN_HOME_INSTR           (uint8_t)0x02

#define ENTRY_MODE_SET_INSTR_BIT    (uint8_t)0x04
#define EMS_ENTRY_DIR_BIT           (uint8_t)0x02
#define EMS_DISP_SHIFT_BIT          (uint8_t)0x01

#define DISPLAY_CONTROL_INSTR_BIT   (uint8_t)0x08
#define DC_DISP_ON_BIT              (uint8_t)0x04
#define DC_CURSOR_ON_BIT            (uint8_t)0x02
#define DC_BLINK_ON_BIT             (uint8_t)0x01

#define CURS_DISP_SHIFT_INSTR_BIT   (uint8_t)0x10
#define CDS_SELECT_BIT              (uint8_t)0x08
#define CDS_DIRECTION_BIT           (uint8_t)0x04

#define FUNCTION_SET_INSTR_BIT      (uint8_t)0x20
#define FS_DATA_LENGTH_BIT          (uint8_t)0x10
#define FS_LINES_NUM_BIT            (uint8_t)0x08
#define FS_FONT_SIZE_BIT            (uint8_t)0x04

// enums for setting bits in instructions

typedef enum RegisterSelect_e {
    RS_instr = 0,       // instruction register
    RS_data = RS_BIT    // data register
} RegisterSelect_e;

typedef enum BacklightState_e {
    BL_off = 0,
    BL_on = BL_BIT
} BacklightState_e;


typedef enum EMS_entryDir_e {
    EMS_rightToLeft = 0,
    EMS_leftToRight = EMS_ENTRY_DIR_BIT
} EMS_entryDir_e;

typedef enum EMS_dispShift_e {
    EMS_shiftOff = 0,
    EMS_shiftOn = EMS_DISP_SHIFT_BIT
} EMS_dispShift_e;


typedef enum DC_dispState_e {
    DC_dispOff = 0,
    DC_dispOn = DC_DISP_ON_BIT
} DC_dispState_e;

typedef enum DC_cursorVisibility_e {
    DC_cursorOff = 0,
    DC_cursorOn = DC_CURSOR_ON_BIT
} DC_cursorVisibility_e;

typedef enum DC_cursorBlink_e {
    DC_blinkOff = 0,
    DC_blinkOn = DC_BLINK_ON_BIT
} DC_cursorBlink_e;


typedef enum FS_dataLength_e {
    FS_4bitMode = 0,
    FS_8bitMode = FS_DATA_LENGTH_BIT
} FS_dataLength_e;

typedef enum FS_linesNum_e {
    FS_1line = 0,
    FS_2lines = FS_LINES_NUM_BIT
} FS_linesNum_e;

typedef enum FS_fontSize_e {
    FS_5x8dots = 0,
    FS_5x10dots = FS_FONT_SIZE_BIT
} FS_fontSize_e;


// I2C parameters

I2C_HandleTypeDef* hi2c;
uint8_t address;

// default settings

BacklightState_e        bl = BL_on;
EMS_entryDir_e          EMS_entryDir = EMS_leftToRight;
EMS_dispShift_e         EMS_dispShift = EMS_shiftOff;
DC_dispState_e          DC_dispState = DC_dispOn;
DC_cursorVisibility_e   DC_cursorVis = DC_cursorOn;
DC_cursorBlink_e        DC_cursorBlink = DC_blinkOff;
const FS_dataLength_e   FS_dataLength = FS_4bitMode;
FS_linesNum_e           FS_linesNum = FS_2lines;
FS_fontSize_e           FS_fontSize = FS_5x8dots;

// instructions to be built from individual bits using the above enum-type variables

uint8_t EMS_instr;
uint8_t DC_instr;
uint8_t FS_instr;


void LCDsendByte(RegisterSelect_e rs, uint8_t data) {
    uint8_t buffer[6] = { 0 };

    // upper half
    buffer[0] = (data & 0xf0) | rs | bl;
    buffer[1] = buffer[0] | EN_BIT;
    buffer[2] = buffer[0];
    // lower half
    buffer[3] = (data << 4) | rs | bl;
    buffer[4] = buffer[3] | EN_BIT;
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
    buffer[0] = INIT_8BIT_MODE | bl;
    buffer[1] = buffer[0] | EN_BIT;
    buffer[2] = buffer[0];

    for (uint8_t i = 0; i < 3; i++) {
        HAL_I2C_Master_Transmit(hi2c, address, buffer, 3, HAL_MAX_DELAY);
        HAL_Delay(5);
    }

    buffer[0] = INIT_4BIT_MODE | bl;
    buffer[1] = buffer[0] | EN_BIT;
    buffer[2] = buffer[0];

    HAL_I2C_Master_Transmit(hi2c, address, buffer, 3, HAL_MAX_DELAY);

    // build instructions from individual bits
    
    EMS_instr = ENTRY_MODE_SET_INSTR_BIT | EMS_entryDir | EMS_dispShift;
    DC_instr = DISPLAY_CONTROL_INSTR_BIT | DC_dispState | DC_cursorVis | DC_cursorBlink;
    FS_instr = FUNCTION_SET_INSTR_BIT | FS_dataLength | FS_linesNum | FS_fontSize;

    LCDsendByte(RS_instr, FS_instr);
    LCDsendByte(RS_instr, EMS_instr);
    LCDsendByte(RS_instr, DC_instr);
    LCDsendByte(RS_instr, CLEAR_DISPLAY_INSTR);
}
