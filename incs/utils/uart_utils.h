/**
 * @file uart_utils.h
 * @brief UART communication utilities - PLIB ring buffer wrapper
 * 
 * Provides centralized UART TX functions for:
 * - Debug output (conditional on DEBUG_ level)
 * - GRBL protocol responses (OK, status queries, errors)
 * - General message transmission to UGS/controllers
 * 
 * Architecture:
 * - Uses PLIB UART3 with 1024-byte TX ring buffer
 * - All writes are non-blocking (ISR handles transmission)
 * - For streaming large data, use UART3_WriteCallbackRegister() for flow control
 */

#ifndef UART_UTILS_H
#define UART_UTILS_H

#include "app.h"
#include "gcode_parser.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== INITIALIZATION ========== */

/**
 * @brief Initialize UART utilities
 * Call this once during system initialization
 */
void UART_Initialize(void);

/* ========== OUTPUT FUNCTIONS ========== */

/**
 * @brief Write data to UART TX ring buffer (non-blocking)
 * @param msg Pointer to message buffer
 * @param len Length of message in bytes
 * @return true if all bytes written to ring buffer, false if buffer full
 * 
 * @note PLIB TX ring buffer (1024 bytes) handles transmission via ISR
 * @note For large data streams, check return value and use callbacks if needed
 */
bool UART_Write(const uint8_t* msg, size_t len);

/**
 * @brief Printf-style message formatting and transmission (non-blocking)
 * @param format Printf-style format string
 * @param ... Variable arguments
 * @return true if message written to ring buffer, false if buffer full
 * 
 * Example: UART_Printf("[GCODE] Queued: %s\r\n", command);
 */
bool UART_Printf(const char* format, ...);

/* ========== GRBL PROTOCOL MESSAGE HELPERS ========== */

typedef struct {
    uint32_t ok_attempted;
    uint32_t ok_enqueued;
    uint32_t ok_dropped;
    uint32_t status_attempted;
    uint32_t status_enqueued;
    uint32_t status_dropped;
    uint32_t last_ok_tx_free;
    uint32_t last_status_tx_free;
    uint32_t last_status_len;
} UART_TxStats;

/**
 * @brief Send GRBL "ok\r\n" response
 * @return true if sent, false if TX buffer full
 */
bool UART_SendOK(void);

/**
 * @brief Reset accumulated UART TX instrumentation counters.
 */
void UART_ResetTxStats(void);

/**
 * @brief Read a snapshot of UART TX instrumentation counters.
 * @param out Destination for the current counters.
 */
void UART_GetTxStats(UART_TxStats* out);

/**
 * @brief Enqueue a preformatted GRBL status report and track TX success.
 * @param msg Status message buffer.
 * @param len Length in bytes.
 * @return true if the full report was enqueued, false on short write.
 */
bool UART_WriteStatusReport(const uint8_t* msg, size_t len);

/**
 * @brief UGS pipeline window size (bytes) — used as the UART3 RX ring-buffer
 * free-space gate before sending "ok".
 *
 * UGS hardcodes a 128-byte sliding window: it sends commands until
 * (bytes-in-flight + next-cmd-length) > 128, then waits for "ok" before
 * sending more.  This means each "ok" we send grants UGS permission to
 * transmit up to 128 bytes.  We must therefore ensure the ring has AT LEAST
 * 128 bytes free before sending "ok" — otherwise UGS's burst overflows the
 * ring and silently drops bytes (truncated G-code → parser stall).
 *
 * Ring = 1024 bytes, window = 128 bytes → 8 full bursts fit in the ring.
 * We stream oks freely (allowing up to 7 bursts to accumulate), then hold
 * on the 8th until the ring drains ≥ 128 bytes of free space, then resume.
 */
#define UART_RX_CMD_MAX_BYTES  128u

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


/**
 * * @brief Print GRBL command help messages
 * 
 */
void UART_PrintHelp(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_UTILS_H */
