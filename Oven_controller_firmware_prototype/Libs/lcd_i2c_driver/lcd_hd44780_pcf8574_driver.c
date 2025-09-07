/**
 * @file lcd_hd44780_pcf8574_driver.c
 * @brief HD44780 LCD driver implementation for STM32 using PCF8574 I2C I/O expander module. See lcd_hd44780_pcf8574_driver.h for API details.
 * @author Mateusz Stelmaszyński
 * @copyright Copyright (c) 2025 Mateusz Stelmaszyński. Licensed under the MIT License. See the LICENSE file in the root directory of this repository for details.
 *
 * This file implements a non-blocking, DMA-based driver for character LCDs using the HD44780 controller and PCF8574 I/O expander module. The driver uses a circular queue to buffer instructions. DMA and I2C are handled using the STM's HAL.
 */

#include "lcd_hd44780_pcf8574_driver.h"

 /* HARDWARE ABSTRACTION */

 // interface control bits

#define RS_DATA_REG                 (uint8_t)0x01
#define RS_INSTR_REG                (uint8_t)0
#define EN_BIT                      (uint8_t)0x04
#define BL_ON                       (uint8_t)0x08   // backlight
#define BL_OFF                      (uint8_t)0

// instructions and arguments bits

#define CLEAR_DISPLAY_INSTR         (uint8_t)0x01

#define RETURN_HOME_INSTR           (uint8_t)0x02

#define ENTRY_MODE_SET_INSTR_BIT    (uint8_t)0x04
#define EMS_ENTRY_RTL               (uint8_t)0x02
#define EMS_ENTRY_LTR               (uint8_t)0
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
#define FS_2LINE_MAP                (uint8_t)0x08
#define FS_1LINE_MAP                (uint8_t)0
#define FS_5x10_DOTS                (uint8_t)0x04
#define FS_5x8_DOTS                 (uint8_t)0

#define SET_DDRAM_ADDR_INSTR_BIT    (uint8_t)0x80
#define DDRAM_ADDR_R0C0             (uint8_t)0
#define DDRAM_ADDR_R1C0             (uint8_t)0x40
#define DDRAM_ADDR_R2C0             (uint8_t)0x14
#define DDRAM_ADDR_R3C0             (uint8_t)0x54

// default settings

uint8_t bl = BL_ON;
uint8_t EMS_entryDir = EMS_ENTRY_LTR;
uint8_t EMS_dispShift = EMS_DISP_SHIFT_OFF;
uint8_t DC_dispState = DC_DISP_ON;
uint8_t DC_cursorVisblty = DC_CURSOR_OFF;
uint8_t DC_cursorBlink = DC_BLINK_OFF;
const uint8_t FS_dataLength = FS_4BIT_MODE;
uint8_t FS_numOfLines = FS_2LINE_MAP;
uint8_t FS_fontSize = FS_5x8_DOTS;

// full instructions for entry mode set, display control, and function set

#define EMS_INSTR   ENTRY_MODE_SET_INSTR_BIT | EMS_entryDir | EMS_dispShift
#define DC_INSTR    DISPLAY_CONTROL_INSTR_BIT | DC_dispState | DC_cursorVisblty | DC_cursorBlink
#define FS_INSTR    FUNCTION_SET_INSTR_BIT | FS_dataLength | FS_numOfLines | FS_fontSize

/* I2C info */

I2C_HandleTypeDef* lcdhi2c;
uint8_t lcdAddress;

/* QUEUE SETUP

Overview of enqueuing/dequeueing and transmitting data:

Start: lcdPrintChar(uint8_t c) (or other API function)
    |
    V
Build an appropriate queue entry (data byte along with the RS bit)
    |
    V
Enqueue that entry (unless the queue is full)
    |
    V
Begin flushing the queue (unless it's already flushing or paused)
    |
    |  <-- Continue flushing from HAL_I2C_MasterTxCpltCallback (unless the queue is paused)
    V
Read the queue entry under qReadIdx without dequeueing it (unless the queue is empty)
    |
    V
Prepare the read entry to be transmitted (by splitting in half and adding EN bit pulses, filling a 6-byte buffer)
    |
    V
Initialise the transmission of that buffer via DMA (unless there are persisting I2C errors)
    |
    V
Dequeue the entry (unless the initialisation failed)
    |
    V
End

*/

#define QUEUE_SIZE  32 // queue entry count (one item is 2 bytes)

typedef struct QueueEntry_t {
    uint8_t rs;
    uint8_t data;
} QueueEntry_t;

QueueEntry_t queue[QUEUE_SIZE];
volatile uint8_t qWriteIdx = 0;
volatile uint8_t qReadIdx = 0;
volatile uint8_t qEntryCount = 0;
volatile bool qPaused = false;

LCDStatus_t enq(QueueEntry_t* e) {
    uint32_t tickStart = HAL_GetTick();
    while (qEntryCount >= QUEUE_SIZE) {         // wait if queue is full
        if (qPaused)                            // only the initial check is relevant, but I put it inside the while loop out of convenience; it doesn't really harm the program
            return LCD_QUEUE_PAUSED_AND_FULL;   // no point in waiting for timeout
        if (HAL_GetTick() - tickStart > 10) {
            qPaused = true;
            return LCD_QUEUE_FULL_TIMEOUT;
        }
    }
    queue[qWriteIdx] = *e;
    qWriteIdx = (qWriteIdx + 1) % QUEUE_SIZE;
    qEntryCount++;
    return LCD_OK;
}

LCDStatus_t qPeek(QueueEntry_t* e) {
    if (qEntryCount == 0) return LCD_QUEUE_EMPTY;
    *e = queue[qReadIdx];
    return LCD_OK;
}

LCDStatus_t deq(void) {
    if (qEntryCount == 0) return LCD_QUEUE_EMPTY;
    qReadIdx = (qReadIdx + 1) % QUEUE_SIZE;
    qEntryCount--;
    return LCD_OK;
}

/* HANDLING AND SENDING DATA */

volatile bool flushInProgress = false;
volatile bool i2cErrorPending = false;
volatile LCDStatus_t lcdStatus = LCD_OK;

LCDStatus_t txByte(uint8_t rs, uint8_t data) {
    static uint8_t buffer[6] = { 0 };
    // upper half
    buffer[0] = (data & 0xf0) | rs | bl;
    buffer[1] = buffer[0] | EN_BIT;
    buffer[2] = buffer[0];
    // lower half
    buffer[3] = (data << 4) | rs | bl;
    buffer[4] = buffer[3] | EN_BIT;
    buffer[5] = buffer[3];

    if (HAL_I2C_GetError(lcdhi2c) != HAL_I2C_ERROR_NONE) {  // if an error persists over 2 transmissions, pause the queue
        if (i2cErrorPending) {
            qPaused = true;
            i2cErrorPending = false;
            return LCD_I2C_ERROR;
        }
        i2cErrorPending = true;
    } else
        i2cErrorPending = false;

    if (HAL_I2C_Master_Transmit_DMA(lcdhi2c, lcdAddress, buffer, 6) != HAL_OK) {
        return LCD_I2C_TX_INIT_FAIL;
    }
    return LCD_OK;
}

LCDStatus_t flush(void) {
    QueueEntry_t e;
    LCDStatus_t status = qPeek(&e);
    if (status == LCD_OK) {
        status = txByte(e.rs, e.data);
        if (status == LCD_OK) {
            deq();
            return status;
        }
    }
    flushInProgress = false;
    return status;
}

LCDStatus_t enqAndBeginFlushing(QueueEntry_t* e) {
    LCDStatus_t status = enq(e);
    if (status != LCD_OK) {
        lcdStatus = status;
        return status;
    }
    if (flushInProgress || qPaused) {
        lcdStatus = LCD_OK;
        return LCD_OK;
    }
    flushInProgress = true;
    lcdStatus = flush();
    return lcdStatus;
}

/* Functions for HAL callbacks */

void lcdFlushQueue(void) {
    if (qPaused) {
        flushInProgress = false;
        lcdStatus = LCD_OK;
        return;
    }
    lcdStatus = flush();
}

void lcdi2cErrorHandler(void) { // prevents queue stalling if HAL_I2C_MasterTxCpltCallback doesn't get called
    flushInProgress = false;
}

/* API - queue control and status */

void lcdQueuePause(void) {
    qPaused = true;
}

LCDStatus_t lcdQueueResume(void) {
    qPaused = false;
    if (flushInProgress) return LCD_OK;
    flushInProgress = true;
    return flush();
}

bool lcdQueueIsPaused(void) {
    return qPaused && flushInProgress;
}

bool lcdQueueIsFull(void) {
    return qEntryCount >= QUEUE_SIZE;
}

LCDStatus_t lcdGetStatus(void) {
    return lcdStatus;
}

/* API - printing characters and strings */

LCDStatus_t lcdPrintChar(uint8_t c) {
    QueueEntry_t e = { RS_DATA_REG, c };
    return enqAndBeginFlushing(&e);
}

LCDStatus_t lcdPrintStr(char* str) {
    LCDStatus_t status;
    while (*str) {
        status = lcdPrintChar(*str++);
        if (status != LCD_OK) break;
    }
    return status;
}

/* API - settings and actions */

// backlight

void lcdSetBacklight(bool state) {
    if (state)
        bl = BL_ON;
    else
        bl = BL_OFF;
}

LCDStatus_t lcdSetBacklightNow(bool state) {
    if (state)
        bl = BL_ON;
    else
        bl = BL_OFF;
    QueueEntry_t e = { RS_INSTR_REG, 0 };
    return enqAndBeginFlushing(&e);
}

// clear display, return home

LCDStatus_t lcdClear(void) {
    QueueEntry_t e = { RS_INSTR_REG, CLEAR_DISPLAY_INSTR };
    LCDStatus_t status = enq(&e);
    if (status != LCD_OK) return status;
    // dummy entries to simulate a delay on the I2C bus (amount might require tweaking)
    e.data = 0;
    for (uint8_t i = 0; i < 3; i++) {
        status = enq(&e);
        if (status != LCD_OK) return status;
    }
    return enqAndBeginFlushing(&e);
}

LCDStatus_t lcdReturnHome(void) {
    QueueEntry_t e = { RS_INSTR_REG, RETURN_HOME_INSTR };
    LCDStatus_t status = enq(&e);
    if (status != LCD_OK) return status;
    // dummy entries to simulate a delay on the I2C bus (amount might require tweaking)
    e.data = 0;
    for (uint8_t i = 0; i < 3; i++) {
        status = enq(&e);
        if (status != LCD_OK) return status;
    }
    return enqAndBeginFlushing(&e);
}

// entry mode set

LCDStatus_t lcdSetLTR(void) {
    EMS_entryDir = EMS_ENTRY_LTR;
    QueueEntry_t e = { RS_INSTR_REG, EMS_INSTR };
    return enqAndBeginFlushing(&e);
}

LCDStatus_t lcdSetRTL(void) {
    EMS_entryDir = EMS_ENTRY_RTL;
    QueueEntry_t e = { RS_INSTR_REG, EMS_INSTR };
    return enqAndBeginFlushing(&e);
}

LCDStatus_t lcdSetAutoScroll(bool state) {
    if (state)
        EMS_dispShift = EMS_DISP_SHIFT_ON;
    else
        EMS_dispShift = EMS_DISP_SHIFT_OFF;
    QueueEntry_t e = { RS_INSTR_REG, EMS_INSTR };
    return enqAndBeginFlushing(&e);
}

// display control

LCDStatus_t lcdSetDisplay(bool state) {
    if (state)
        DC_dispState = DC_DISP_ON;
    else
        DC_dispState = DC_DISP_OFF;
    QueueEntry_t e = { RS_INSTR_REG, DC_INSTR };
    return enqAndBeginFlushing(&e);
}

LCDStatus_t lcdSetCursorVisible(bool state) {
    if (state)
        DC_cursorVisblty = DC_CURSOR_ON;
    else
        DC_cursorVisblty = DC_CURSOR_OFF;
    QueueEntry_t e = { RS_INSTR_REG, DC_INSTR };
    return enqAndBeginFlushing(&e);
}

LCDStatus_t lcdSetCursorBlink(bool state) {
    if (state)
        DC_cursorBlink = DC_BLINK_ON;
    else
        DC_cursorBlink = DC_BLINK_OFF;
    QueueEntry_t e = { RS_INSTR_REG, DC_INSTR };
    return enqAndBeginFlushing(&e);
}

// cursor/display shift

LCDStatus_t lcdShiftCursorR(void) {
    QueueEntry_t e = { RS_INSTR_REG, CURS_DISP_SHIFT_INSTR_BIT | CDS_SHIFT_CURSOR | CDS_SHIFT_RIGHT };
    return enqAndBeginFlushing(&e);
}

LCDStatus_t lcdShiftCursorL(void) {
    QueueEntry_t e = { RS_INSTR_REG, CURS_DISP_SHIFT_INSTR_BIT | CDS_SHIFT_CURSOR | CDS_SHIFT_LEFT };
    return enqAndBeginFlushing(&e);
}

LCDStatus_t lcdShiftDisplayR(void) {
    QueueEntry_t e = { RS_INSTR_REG, CURS_DISP_SHIFT_INSTR_BIT | CDS_SHIFT_DISPLAY | CDS_SHIFT_RIGHT };
    return enqAndBeginFlushing(&e);
}

LCDStatus_t lcdShiftDisplayL(void) {
    QueueEntry_t e = { RS_INSTR_REG, CURS_DISP_SHIFT_INSTR_BIT | CDS_SHIFT_DISPLAY | CDS_SHIFT_LEFT };
    return enqAndBeginFlushing(&e);
}

// set DDRAM address (cursor position)

LCDStatus_t lcdSetCursorPos(uint8_t row, uint8_t col) {
    QueueEntry_t e;
    e.rs = RS_INSTR_REG;
    switch (row) {
        case 0:
            e.data = (SET_DDRAM_ADDR_INSTR_BIT | DDRAM_ADDR_R0C0) + col;
            break;
        case 1:
            e.data = (SET_DDRAM_ADDR_INSTR_BIT | DDRAM_ADDR_R1C0) + col;
            break;
        case 2:
            e.data = (SET_DDRAM_ADDR_INSTR_BIT | DDRAM_ADDR_R2C0) + col;
            break;
        case 3:
            e.data = (SET_DDRAM_ADDR_INSTR_BIT | DDRAM_ADDR_R3C0) + col;
            break;
        default:
            break;
    }
    return enqAndBeginFlushing(&e);
}

/* LCD INITIALISATION */

#define INIT_8BIT_MODE 0x30
#define INIT_4BIT_MODE 0x20

LCDStatus_t lcdInit(I2C_HandleTypeDef* hi2c, uint8_t address, uint8_t numOfLines, uint8_t cellHeight, bool backlight) {
    lcdhi2c = hi2c;
    lcdAddress = address << 1;

    if (numOfLines > 1)
        FS_numOfLines = FS_2LINE_MAP;
    else
        FS_numOfLines = FS_1LINE_MAP;

    if (cellHeight == 10)
        FS_fontSize = FS_5x10_DOTS;
    else
        FS_fontSize = FS_5x8_DOTS;

    if (backlight)
        bl = BL_ON;
    else
        bl = BL_OFF;

    /*
    initialisation sequence:
    1. send INIT_8BIT_MODE 3 times to enforce 8-bit data length whatever the previous state was
    2. send INIT_4BIT_MODE once to switch to 4-bit data length
    3. send display settings (function set, entry mode set, display control)
    4. clear display
    */

    uint8_t buffer[3] = { 0 };
    buffer[0] = INIT_8BIT_MODE | bl;
    buffer[1] = buffer[0] | EN_BIT;
    buffer[2] = buffer[0];

    for (uint8_t i = 0; i < 3; i++) {
        if (HAL_I2C_Master_Transmit(lcdhi2c, lcdAddress, buffer, 3, 1000) != HAL_OK)
            return LCD_I2C_TX_INIT_FAIL;
        HAL_Delay(5);
    }

    buffer[0] = INIT_4BIT_MODE | bl;
    buffer[1] = buffer[0] | EN_BIT;
    buffer[2] = buffer[0];

    if (HAL_I2C_Master_Transmit(lcdhi2c, lcdAddress, buffer, 3, 1000) != HAL_OK)
        return LCD_I2C_TX_INIT_FAIL;

    QueueEntry_t e;
    e.rs = RS_INSTR_REG;
    e.data = FS_INSTR;
    enq(&e);
    e.data = EMS_INSTR;
    enq(&e);
    e.data = DC_INSTR;
    enq(&e);

    flushInProgress = true;
    LCDStatus_t status = flush();
    if (status != LCD_OK) return status;

    return lcdClear();
}
