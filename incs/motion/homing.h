#include "../data_structures.h"
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




// ===== HOMING API =====

/**
 * @brief Initialize homing system
 * Must be called once during startup
 */

void HOMING_Initialize(void);
void HOMING_Reset(void);
bool HOMING_Start(uint32_t axes_mask);
HomingState HOMING_Tasks(void);
void HOMING_Abort(void);
bool HOMING_IsActive(void);
HomingState HOMING_GetState(void);
void HOMING_ClearAlarm(void);
void HOMING_StartLocate(void);
void HOMING_StartPulloff(void);
bool HOMING_LimitTriggered(void);
bool HOMING_NextAxis(void);

#endif // HOMING_H
