/**
 * @file homing.h
 * @brief GRBL v1.1 compatible homing cycle implementation
 * 
 * Implements three-phase homing sequence:
 * 1. SEEK phase: Fast approach to limit switch
 * 2. LOCATE phase: Slow precision positioning
 * 3. PULLOFF phase: Retract to clear switch
 * 
 * Features:
 * - Multi-axis homing (sequential or simultaneous)
 * - Configurable direction per axis ($23 mask)
 * - Debounce delay for switch stability
 * - Alarm on homing failure
 */

#ifndef HOMING_H
#define HOMING_H

#include <stdint.h>
#include <stdbool.h>
#include "data_structures.h"

// Homing state machine
typedef enum {
    HOMING_STATE_IDLE = 0,      // No homing in progress
    HOMING_STATE_SEEK,          // Fast approach to limit
    HOMING_STATE_LOCATE,        // Slow precision homing
    HOMING_STATE_PULLOFF,       // Retract from limit
    HOMING_STATE_COMPLETE,      // Homing successful
    HOMING_STATE_ALARM          // Homing failed
} HomingState;

// Homing phase tracking
typedef struct {
    HomingState state;
    E_AXIS current_axis;        // Axis currently being homed
    uint32_t axes_to_home;      // Bitmask of axes to home (bit 0=X, 1=Y, 2=Z, 3=A)
    uint32_t axes_homed;        // Bitmask of completed axes
    
    // Timing
    uint32_t debounce_start;    // CoreTimer value when limit triggered
    bool debouncing;            // True during debounce delay
    
    // Motion tracking
    bool motion_active;         // True when homing move in progress
    uint32_t alarm_code;        // Non-zero if homing failed
} HomingControl;

// Global homing control (single instance)
extern HomingControl g_homing;

// ===== HOMING API =====

/**
 * @brief Initialize homing system
 * Must be called once during startup
 */
void HOMING_Initialize(void);

/**
 * @brief Start homing cycle for specified axes
 * @param axes_mask Bitmask of axes to home (0x01=X, 0x02=Y, 0x04=Z, 0x08=A)
 * @return true if homing started, false if already active or disabled
 * 
 * Example: HOMING_Start(0x07) homes X, Y, Z axes
 */
bool HOMING_Start(uint32_t axes_mask);

/**
 * @brief Execute homing state machine (call in main loop)
 * @return Current homing state
 */
HomingState HOMING_Tasks(void);

/**
 * @brief Abort homing cycle immediately
 * Sets alarm state, stops motion, disables steppers
 */
void HOMING_Abort(void);

/**
 * @brief Check if homing is currently active
 * @return true if homing in progress
 */
bool HOMING_IsActive(void);

/**
 * @brief Get current homing state
 * @return HomingState enum value
 */
HomingState HOMING_GetState(void);

/**
 * @brief Clear homing alarm and return to idle
 * Call after user acknowledges homing failure
 */
void HOMING_ClearAlarm(void);

// ===== INTERNAL HELPERS (used by state machine) =====

/**
 * @brief Start seek phase for current axis
 * Fast approach at homing_seek_rate
 */
void HOMING_StartSeek(void);

/**
 * @brief Start locate phase for current axis
 * Slow precision move at homing_feed_rate
 */
void HOMING_StartLocate(void);

/**
 * @brief Start pulloff phase for current axis
 * Retract by homing_pull_off distance
 */
void HOMING_StartPulloff(void);

/**
 * @brief Check if limit switch triggered for current axis
 * Respects invert mask and debounce delay
 * @return true if limit triggered and debounced
 */
bool HOMING_LimitTriggered(void);

/**
 * @brief Move to next axis in homing sequence
 * @return true if more axes to home, false if complete
 */
bool HOMING_NextAxis(void);

#endif // HOMING_H
