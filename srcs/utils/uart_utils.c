/**
 * @file uart_utils.c
 * @brief UART communication utilities - PLIB ring buffer wrapper
 * @note PLIB TX ring buffer (1024 bytes) handles transmission in background via ISR
 * 
 * Architecture:
 * - UART3_Write() copies data to 1024-byte TX ring buffer (non-blocking)
 * - Returns immediately - ISR handles actual transmission
 * - Can check UART3_WriteCountGet() to monitor buffer usage
 * - Can use callbacks to refill buffer when transmission completes
 */

#include "utils/uart_utils.h"
#include "config/default/peripheral/uart/plib_uart3.h"
#include "data_structures.h"
#include "common.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* ========== INITIALIZATION ========== */

static char buffer[512];  // Static to avoid stack overflow
static UART_TxStats s_uart_tx_stats;

void UART_ResetTxStats(void) {
    memset(&s_uart_tx_stats, 0, sizeof(s_uart_tx_stats));
}

void UART_GetTxStats(UART_TxStats* out) {
    if (out == NULL) {
        return;
    }
    *out = s_uart_tx_stats;
}

void UART_Initialize(void) {
    // ✅ PLIB ring buffer handles TX/RX automatically via ISR
    // No callbacks needed for normal operation (1024-byte buffer is large enough)
    // For streaming large data, register callback via UART3_WriteCallbackRegister()
    UART_ResetTxStats();
}

/* ========== FIRE-AND-FORGET OUTPUT FUNCTIONS ========== */

bool UART_Write(const uint8_t* msg, size_t len) {
    if (msg == NULL || len == 0) {
        return false;  // Invalid parameters
    }
    size_t written = UART3_Write((uint8_t*)msg, len);
    return (written == len);  // True if all bytes copied to ring buffer
}

bool UART_Printf(const char* format, ...) {
    if (format == NULL) {
        return false;
    }
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (len <= 0 || len >= (int)sizeof(buffer)) {
        return false;
    }
    return UART_Write((uint8_t*)buffer, (size_t)len);
}

/* ========== GRBL PROTOCOL MESSAGE HELPERS ========== */

bool UART_SendOK(void) {
    size_t written;

    s_uart_tx_stats.ok_attempted++;
    s_uart_tx_stats.last_ok_tx_free = (uint32_t)UART3_WriteFreeBufferCountGet();

    written = UART3_Write((uint8_t*)"ok\r\n", 4);
    if (written == 4u) {
        s_uart_tx_stats.ok_enqueued++;
        return true;
    }

    s_uart_tx_stats.ok_dropped++;
    return false;
}

bool UART_WriteStatusReport(const uint8_t* msg, size_t len) {
    size_t written;

    if (msg == NULL || len == 0u) {
        return false;
    }

    s_uart_tx_stats.status_attempted++;
    s_uart_tx_stats.last_status_tx_free = (uint32_t)UART3_WriteFreeBufferCountGet();
    s_uart_tx_stats.last_status_len = (uint32_t)len;

    written = UART3_Write((uint8_t*)msg, len);
    if (written == len) {
        s_uart_tx_stats.status_enqueued++;
        return true;
    }

    s_uart_tx_stats.status_dropped++;
    return false;
}

bool UART_SendGrblStatus(const char* state, 
                         float mpos_x, float mpos_y, float mpos_z,
                         float wpos_x, float wpos_y, float wpos_z,
                         float feedrate, uint32_t spindle_rpm) {
    if (state == NULL) {
        return false;
    }
    int len = snprintf(buffer, sizeof(buffer),
                       "<%s|MPos:%.3f,%.3f,%.3f|WPos:%.3f,%.3f,%.3f|FS:%.0f,%.0f>\r\n",
                       state, mpos_x, mpos_y, mpos_z, wpos_x, wpos_y, wpos_z,
                       feedrate, (float)spindle_rpm);
    if (len <= 0 || len >= (int)sizeof(buffer)) {
        return false;
    }
    return UART_WriteStatusReport((uint8_t*)buffer, (size_t)len);
}

bool UART_SendError(uint8_t error_code) {
    return UART_Printf("error:%u\r\n", error_code);
}

bool UART_SendAlarm(uint8_t alarm_code) {
    return UART_Printf("ALARM:%u\r\n", alarm_code);
}

bool UART_SendMessage(const char* msg) {
    if (msg == NULL) {
        return false;
    }
    return UART_Write((uint8_t*)msg, strlen(msg));
}

void UART_PrintHelp(void)
{
    UART3_Write((uint8_t*)"[HLP:$$ $# $G $I $N $X $H $SLP $C $J=line $RST=x]\r\n", 52);
    UART3_Write((uint8_t*)"[HLP:G0 G1 G2 G3 G4 G17 G18 G19 G20 G21 G28 G30 G90 G91 G92 G93 G94]\r\n", 69);
    UART3_Write((uint8_t*)"[HLP:M0 M2 M3 M4 M5 M7 M8 M9 M30]\r\n", 35);
    UART3_Write((uint8_t*)"[HLP:F S T]\r\n", 13);
    UART_SendOK();
}

/* ========== SOFT RESET (Ctrl+X) ========================================== */
/* NOTE: Soft reset logic has been consolidated into GCODE_SoftReset() in gcode_parser.c */
/* This eliminates the circular dependency and keeps all reset logic in one place */