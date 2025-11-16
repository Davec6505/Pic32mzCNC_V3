#ifndef STEPPER_H
#define STEPPER_H

#include <stdint.h>
#include <stdbool.h>
#include "common.h"
#include "data_structures.h"  // APP_DATA, MotionSegment defined here

// ✅ ARRAY-BASED: Position tracking structure
typedef struct {
    int32_t steps[NUM_AXIS];          // Step counters [X, Y, Z, A] - signed, can go negative
    float steps_per_mm[NUM_AXIS];     // Steps per mm for linear axes [X, Y, Z, A]
} StepperPosition;

// ============================================================================
// Public API
// ============================================================================

// ✅ HARD LIMIT ALARM FLAG - Shared between ISR and main loop
// Set by stepper ISR when hard limit triggered, cleared by $X command
extern volatile bool g_hard_limit_alarm;

void STEPPER_Initialize(APP_DATA* appData);
void STEPPER_LoadSegment(MotionSegment* segment);         // Load new segment for execution
void STEPPER_SetStepRate(uint32_t rate_ticks);            // Update PR2 for velocity profiling
void STEPPER_SetDirection(E_AXIS axis, bool forward);     // Set direction for axis
void STEPPER_DisableAll(void);                            // Emergency stop
StepperPosition* STEPPER_GetPosition(void);               // Get current position (snapshot)
StepperPosition* STEPPER_GetPositionPointer(void);        // Get pointer to live position counters

/* Returns true if steppers currently enabled. */
bool STEPPER_IsEnabled(void);

/* Enable all stepper drivers (counterpart to STEPPER_DisableAll). */
void STEPPER_EnableAll(void);

/* Check if dwell timer has completed (for DWELL segments). */
bool STEPPER_IsDwellComplete(void);

/* Reload cached settings from flash (call after changing $0-$5 settings). */
void STEPPER_ReloadSettings(void);

#endif /* STEPPER_H */