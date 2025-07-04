/**
 * @file lcd_hd44780_pcf8574_driver.c
 * @brief HD44780 LCD driver implementation for STM32 using PCF8574 I2C I/O expander module.
 * @author Mateusz Stelmaszyński
 * @copyright Copyright (c) 2025 Mateusz Stelmaszyński. Licensed under the MIT License. See the LICENSE file in the \software\ directory of this repository for full terms.
 *
 * This file implements a non-blocking, DMA-based driver for character LCDs using the HD44780 controller and PCF8574 I/O expander module. The driver uses a circular queue to buffer characters and instructions. The DMA and I2C interface is handled using the STM's HAL.
 *
 * See lcd_hd44780_pcf8574_driver.h for API details.
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
uint8_t EMS_entryDir = EMS_ENTRY_TO_RIGHT;
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

/* CIRCULAR QUEUE SETUP

General flow of queuing and sending data:

lcdPrintChar(uint8_t c) or other API function:
1. Builds an appropriate queue entry (data byte along with the RS bit) and puts it in the circular queue
2. Checks whether any transmissions are already in progress
A. If not:
3A. Sets the txInProgress flag
4A. Calls sendOldestQueueEntry()
B. If yes:
3B. Returns

sendOldestQueueEntry():
1. Tries to dequeue the oldest entry
A. On failure (queue empty):
2A. Clears the txInProgress flag
3A. Returns
B. On success:
2B. Passes the dequeued entry to sendByte(uint8_t rs, uint8_t data)

sendByte(uint8_t rs, uint8_t data)
1. Prepares the received queue entry to be sent (by splitting in halve and adding EN bit pulses, 6 bytes in total)
2. Initialises the transmission of these 6 bytes by calling HAL_I2C_Master_Transmit_DMA(...)

HAL_I2C_MasterTxCpltCallback(...) (after each sent block):
1. Calls sendOldestQueueEntry()
*/

#define QUEUE_SIZE  32 // queue entry count (one item is 2 bytes)

typedef struct QueueEntry {
    uint8_t rs;
    uint8_t data;
} QueueEntry;

QueueEntry queue[QUEUE_SIZE];
volatile uint8_t qWriteIdx = 0;
volatile uint8_t qReadIdx = 0;
volatile uint8_t qCurrEntryCnt = 0;

void enqueue(QueueEntry* entry) {
    while (qCurrEntryCnt >= QUEUE_SIZE);   // wait if queue is full
    queue[qWriteIdx].rs = entry->rs;
    queue[qWriteIdx].data = entry->data;
    qWriteIdx = (qWriteIdx + 1) % QUEUE_SIZE;
    qCurrEntryCnt++;
}

bool dequeue(QueueEntry* entry) {
    if (qCurrEntryCnt == 0)
        return 1;     // queue is empty
    entry->rs = queue[qReadIdx].rs;
    entry->data = queue[qReadIdx].data;
    qReadIdx = (qReadIdx + 1) % QUEUE_SIZE;
    qCurrEntryCnt--;
    return 0;
}

/* HANDLING AND SENDING DATA */

volatile bool txInProgress = false;

void sendByte(uint8_t rs, uint8_t data) {
    static uint8_t buffer[6] = { 0 };
    // upper half
    buffer[0] = (data & 0xf0) | rs | bl;
    buffer[1] = buffer[0] | EN_BIT;
    buffer[2] = buffer[0];
    // lower half
    buffer[3] = (data << 4) | rs | bl;
    buffer[4] = buffer[3] | EN_BIT;
    buffer[5] = buffer[3];

    HAL_I2C_Master_Transmit_DMA(lcdhi2c, lcdAddress, buffer, 6);
}

void sendOldestQueueEntry(void) {
    QueueEntry entry;
    if (dequeue(&entry)) {
        txInProgress = false;
        return;
    }
    sendByte(entry.rs, entry.data);
}

void enqueueAndSendOldestEntry(QueueEntry* entry) {
    enqueue(entry);
    if (!txInProgress) {
        txInProgress = true;
        sendOldestQueueEntry();
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c) {
    if (hi2c == lcdhi2c) {
        sendOldestQueueEntry();
    }
}

/* API - printing characters and strings */

void lcdPrintChar(uint8_t c) {
    QueueEntry entry = { RS_DATA_REG, c };
    enqueueAndSendOldestEntry(&entry);
}

void lcdPrintStr(char* str) {
    while (*str)
        lcdPrintChar(*str++);
}

/* API - settings and actions */

// backlight

void lcdBacklightOn(bool state) {
    if (state)
        bl = BL_ON;
    else
        bl = BL_OFF;
}

void lcdBacklightOnImmediate(bool state) {
    if (state)
        bl = BL_ON;
    else
        bl = BL_OFF;
    QueueEntry entry = { RS_INSTR_REG, 0 };
    enqueueAndSendOldestEntry(&entry);
}

// clear display, return home

void lcdClear(void) {
    QueueEntry entry = { RS_INSTR_REG, CLEAR_DISPLAY_INSTR };
    enqueue(&entry);

    // dummy entries to simulate delay (amount might require tweaking)
    entry.data = 0;
    for (uint8_t i = 0; i < 3; i++) {
        enqueue(&entry);
    }

    enqueueAndSendOldestEntry(&entry);
}

void lcdReturnHome(void) {
    QueueEntry entry = { RS_INSTR_REG, RETURN_HOME_INSTR };
    enqueue(&entry);

    // dummy entries to simulate delay (amount might require tweaking)
    entry.data = 0;
    for (uint8_t i = 0; i < 3; i++) {
        enqueue(&entry);
    }

    enqueueAndSendOldestEntry(&entry);
}

// entry mode set

void lcdPrintLeftToRight(void) {
    EMS_entryDir = EMS_ENTRY_TO_RIGHT;
    QueueEntry entry = { RS_INSTR_REG, EMS_INSTR };
    enqueueAndSendOldestEntry(&entry);
}

void lcdPrintRightToLeft(void) {
    EMS_entryDir = EMS_ENTRY_TO_LEFT;
    QueueEntry entry = { RS_INSTR_REG, EMS_INSTR };
    enqueueAndSendOldestEntry(&entry);
}

void lcdAutoScroll(bool state) {
    if (state)
        EMS_dispShift = EMS_DISP_SHIFT_ON;
    else
        EMS_dispShift = EMS_DISP_SHIFT_OFF;
    QueueEntry entry = { RS_INSTR_REG, EMS_INSTR };
    enqueueAndSendOldestEntry(&entry);
}

// display control

void lcdDisplayOn(bool state) {
    if (state)
        DC_dispState = DC_DISP_ON;
    else
        DC_dispState = DC_DISP_OFF;
    QueueEntry entry = { RS_INSTR_REG, DC_INSTR };
    enqueueAndSendOldestEntry(&entry);
}

void lcdCursorVisible(bool state) {
    if (state)
        DC_cursorVisblty = DC_CURSOR_ON;
    else
        DC_cursorVisblty = DC_CURSOR_OFF;
    QueueEntry entry = { RS_INSTR_REG, DC_INSTR };
    enqueueAndSendOldestEntry(&entry);
}

void lcdCursorBlink(bool state) {
    if (state)
        DC_cursorBlink = DC_BLINK_ON;
    else
        DC_cursorBlink = DC_BLINK_OFF;
    QueueEntry entry = { RS_INSTR_REG, DC_INSTR };
    enqueueAndSendOldestEntry(&entry);
}

// cursor/display shift

void lcdShiftCursorRight(void) {
    QueueEntry entry = { RS_INSTR_REG, CURS_DISP_SHIFT_INSTR_BIT | CDS_SHIFT_CURSOR | CDS_SHIFT_RIGHT };
    enqueueAndSendOldestEntry(&entry);
}

void lcdShiftCursorLeft(void) {
    QueueEntry entry = { RS_INSTR_REG, CURS_DISP_SHIFT_INSTR_BIT | CDS_SHIFT_CURSOR | CDS_SHIFT_LEFT };
    enqueueAndSendOldestEntry(&entry);
}

void lcdShiftDisplayRight(void) {
    QueueEntry entry = { RS_INSTR_REG, CURS_DISP_SHIFT_INSTR_BIT | CDS_SHIFT_DISPLAY | CDS_SHIFT_RIGHT };
    enqueueAndSendOldestEntry(&entry);
}

void lcdShiftDisplayLeft(void) {
    QueueEntry entry = { RS_INSTR_REG, CURS_DISP_SHIFT_INSTR_BIT | CDS_SHIFT_DISPLAY | CDS_SHIFT_LEFT };
    enqueueAndSendOldestEntry(&entry);
}

// set DDRAM address (cursor position)

void lcdSetCursorPosition(uint8_t row, uint8_t col) {
    QueueEntry entry;
    entry.rs = RS_INSTR_REG;
    switch (row) {
        case 0:
            entry.data = (SET_DDRAM_ADDR_INSTR_BIT | DDRAM_ADDR_R0C0) + col;
            break;
        case 1:
            entry.data = (SET_DDRAM_ADDR_INSTR_BIT | DDRAM_ADDR_R1C0) + col;
            break;
        case 2:
            entry.data = (SET_DDRAM_ADDR_INSTR_BIT | DDRAM_ADDR_R2C0) + col;
            break;
        case 3:
            entry.data = (SET_DDRAM_ADDR_INSTR_BIT | DDRAM_ADDR_R3C0) + col;
            break;
        default:
            break;
    }
    enqueueAndSendOldestEntry(&entry);
}

/* LCD INITIALISATION */

#define INIT_8BIT_MODE 0x30
#define INIT_4BIT_MODE 0x20

void lcdInit(I2C_HandleTypeDef* hi2c, uint8_t address, uint8_t numOfLines, uint8_t cellHeight, bool backlight) {
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
        HAL_I2C_Master_Transmit(lcdhi2c, lcdAddress, buffer, 3, HAL_MAX_DELAY);
        HAL_Delay(5);
    }

    buffer[0] = INIT_4BIT_MODE | bl;
    buffer[1] = buffer[0] | EN_BIT;
    buffer[2] = buffer[0];

    HAL_I2C_Master_Transmit(lcdhi2c, lcdAddress, buffer, 3, HAL_MAX_DELAY);

    QueueEntry entry;
    entry.rs = RS_INSTR_REG;
    entry.data = FS_INSTR;
    enqueue(&entry);
    entry.data = EMS_INSTR;
    enqueue(&entry);
    entry.data = DC_INSTR;
    enqueue(&entry);

    txInProgress = true;
    sendOldestQueueEntry();

    lcdClear();
}
