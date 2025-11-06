/**
 * @file uart_utils.c
 * @brief Non-blocking UART communication utilities implementation
 */

#include "utils/uart_utils.h"
#include "config/default/peripheral/uart/plib_uart3.h"
#include <stdio.h>
#include <string.h>

/* ========== TX READY FLAG ========== */

volatile bool uart3TxReady = true;  // Initially ready

/* ========== UART WRITE CALLBACK ========== */

/**
 * @brief UART3 write event callback - sets TX ready flag when buffer empty
 */
static void usartWriteEventHandler(UART_EVENT event, uintptr_t context) {
    if (event == UART_EVENT_WRITE_THRESHOLD_REACHED) {
        uart3TxReady = true;  // Ready for next transmission
    }
}

/* ========== INITIALIZATION ========== */

void UART_Initialize(void) {
    // Register write callback
    UART3_WriteCallbackRegister(usartWriteEventHandler, (uintptr_t)NULL);
    
    // Enable write notifications (CRITICAL: callback won't fire without this!)
    UART3_WriteNotificationEnable(true, true);
    
    uart3TxReady = true;  // Initially ready
}

/* ========== NON-BLOCKING OUTPUT FUNCTIONS ========== */

bool UART_Write(const uint8_t* msg, size_t len) {
    if (!uart3TxReady || msg == NULL || len == 0) {
        return false;  // TX busy or invalid parameters
    }
    
    uart3TxReady = false;  // Mark as busy
    size_t written = UART3_Write((uint8_t*)msg, len);
    
    if (written == 0) {
        uart3TxReady = true;  // Failed to send, restore ready flag
        return false;
    }
    
    return true;  // Callback will set uart3TxReady = true when complete
}

bool UART_Printf(const char* format, ...) {
    if (!uart3TxReady || format == NULL) {
        return false;  // TX busy or invalid format
    }
    
    static char buffer[256];  // Static to avoid stack overflow
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    if (len <= 0 || len >= (int)sizeof(buffer)) {
        return false;  // Format error or buffer overflow
    }
    
    return UART_Write((uint8_t*)buffer, (size_t)len);
}

/* ========== GRBL PROTOCOL MESSAGE HELPERS ========== */

bool UART_SendOK(void) {
    return UART_Write((uint8_t*)"OK\r\n", 4);
}

bool UART_SendGrblStatus(const char* state, 
                         float mpos_x, float mpos_y, float mpos_z,
                         float wpos_x, float wpos_y, float wpos_z,
                         float feedrate, uint32_t spindle_rpm) {
    if (!uart3TxReady || state == NULL) {
        return false;
    }
    
    static char buffer[256];
    int len = snprintf(buffer, sizeof(buffer),
        "<%s|MPos:%.3f,%.3f,%.3f|WPos:%.3f,%.3f,%.3f|FS:%.0f,%u>\r\n",
        state, mpos_x, mpos_y, mpos_z, wpos_x, wpos_y, wpos_z,
        feedrate, spindle_rpm);
    
    if (len <= 0 || len >= (int)sizeof(buffer)) {
        return false;
    }
    
    return UART_Write((uint8_t*)buffer, (size_t)len);
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
