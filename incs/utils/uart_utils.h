/**
 * @file uart_utils.h
 * @brief Non-blocking UART communication utilities for debug and protocol messages
 * 
 * Provides centralized, non-blocking UART TX functions for:
 * - Debug output (conditional on DEBUG_ level)
 * - GRBL protocol responses (OK, status queries, errors)
 * - General message transmission to UGS/controllers
 * 
 * Uses callback-driven TX complete detection to prevent blocking during motion.
 */

#ifndef UART_UTILS_H
#define UART_UTILS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== NON-BLOCKING UART TX STATE ========== */

/**
 * @brief TX ready flag - true when UART3 TX buffer empty, false when busy
 * Set by UART write callback, checked before sending to avoid blocking
 */
extern volatile bool uart3TxReady;

/* ========== INITIALIZATION ========== */

/**
 * @brief Initialize UART utilities (register callback, enable notifications)
 * Call this once during system initialization
 */
void UART_Initialize(void);

/* ========== NON-BLOCKING OUTPUT FUNCTIONS ========== */

/**
 * @brief Non-blocking UART write - only sends if TX ready
 * @param msg Pointer to message buffer
 * @param len Length of message in bytes
 * @return true if message sent, false if TX busy (message dropped)
 */
bool UART_Write(const uint8_t* msg, size_t len);

/**
 * @brief Non-blocking printf-style message formatting and transmission
 * @param format Printf-style format string
 * @param ... Variable arguments
 * @return true if message sent, false if TX busy (message dropped)
 * 
 * Example: UART_Printf("[GCODE] Queued: %s\r\n", command);
 */
bool UART_Printf(const char* format, ...);

/**
 * @brief Check if UART3 is ready to send
 * @return true if ready, false if TX busy
 */
static inline bool UART_IsReady(void) {
    return uart3TxReady;
}

/* ========== GRBL PROTOCOL MESSAGE HELPERS ========== */

/**
 * @brief Send GRBL "OK\r\n" response
 * @return true if sent, false if TX busy
 */
bool UART_SendOK(void);

/**
 * @brief Send GRBL status report (e.g., "<Idle|MPos:0.000,0.000,0.000|...>")
 * @param state State string ("Idle", "Run", "Hold", "Alarm")
 * @param mpos_x Machine X position
 * @param mpos_y Machine Y position
 * @param mpos_z Machine Z position
 * @param wpos_x Work X position
 * @param wpos_y Work Y position
 * @param wpos_z Work Z position
 * @param feedrate Current feedrate
 * @param spindle_rpm Current spindle RPM
 * @return true if sent, false if TX busy
 */
bool UART_SendGrblStatus(const char* state, 
                         float mpos_x, float mpos_y, float mpos_z,
                         float wpos_x, float wpos_y, float wpos_z,
                         float feedrate, uint32_t spindle_rpm);

/**
 * @brief Send GRBL error message (e.g., "error:33\r\n")
 * @param error_code GRBL error code (1-99)
 * @return true if sent, false if TX busy
 */
bool UART_SendError(uint8_t error_code);

/**
 * @brief Send GRBL alarm message (e.g., "ALARM:2\r\n")
 * @param alarm_code GRBL alarm code
 * @return true if sent, false if TX busy
 */
bool UART_SendAlarm(uint8_t alarm_code);

/**
 * @brief Send custom message (general purpose)
 * @param msg Null-terminated string
 * @return true if sent, false if TX busy
 */
bool UART_SendMessage(const char* msg);

#ifdef __cplusplus
}
#endif

#endif /* UART_UTILS_H */
