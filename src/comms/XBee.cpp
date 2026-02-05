/**
 * XBee communication for CanSat 2026.
 * Hardware: XBee-Pro XSC 900 MHz (wire antenna on CanSat), SparkFun XBee Explorer Regulated.
 * Teensy 4.1 connects to Explorer via UART: Serial1 (RX = pin 0, TX = pin 1).
 * Transparent serial mode: bytes sent are transmitted over the air to the other XBee.
 */
#include "XBee.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <Arduino.h>

// Serial port for XBee (Teensy 4.1: Serial1 = RX pin 0, TX pin 1)
#define XBEE_SERIAL Serial1

// XBee-Pro XSC 900 MHz default baud is 9600. Change if your radios are configured differently.
#define XBEE_BAUD 9600

// Command/telemetry lines end with \r or \n
#define XBEE_LINE_BUF_SIZE 256

static bool xbeeReadyFlag = false;

// Line buffer for receiving commands (GCS sends "CMD,1057,CX,ON\r\n" etc.)
static char lineBuf[XBEE_LINE_BUF_SIZE];
static size_t lineIdx = 0;

void initXBee() {
    XBEE_SERIAL.begin(XBEE_BAUD);
    lineIdx = 0;
    lineBuf[0] = '\0';
    xbeeReadyFlag = false;
    // Allow UART and XBee to stabilize (Explorer and radio power-up)
    delay(50);
    xbeeReadyFlag = true;
}

bool xbeeReady() {
    return xbeeReadyFlag;
}

void xbeeSend(const uint8_t* data, size_t length) {
    if (data == nullptr || length == 0) return;
    if (!xbeeReadyFlag) return;
    XBEE_SERIAL.write(data, length);
}

/**
 * Non-blocking receive: returns one complete line (terminated by \r or \n).
 * Call repeatedly from loop; when a full line has been received, returns true
 * and the line (without \r\n) is in buffer, length set. Buffer is null-terminated.
 */
bool xbeeReceive(uint8_t* buffer, size_t* length) {
    if (buffer == nullptr || length == nullptr || *length == 0) return false;

    while (XBEE_SERIAL.available()) {
        int c = XBEE_SERIAL.read();
        if (c < 0) break;

        if (c == '\r' || c == '\n') {
            if (lineIdx > 0) {
                lineBuf[lineIdx] = '\0';
                size_t copyLen = lineIdx;
                if (copyLen >= *length) copyLen = *length - 1;
                memcpy(buffer, lineBuf, copyLen);
                buffer[copyLen] = '\0';
                *length = copyLen;
                lineIdx = 0;
                return true;
            }
            continue;
        }

        if (lineIdx < XBEE_LINE_BUF_SIZE - 1) {
            lineBuf[lineIdx++] = (char)c;
        } else {
            // Overflow: discard and reset
            lineIdx = 0;
        }
    }

    return false;
}

void updateXBee() {
    // Optional: could poll for errors or buffer full; transparent mode has no link layer.
    // xbeeReadyFlag remains true once init succeeded.
}
