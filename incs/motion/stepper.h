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

// ✅ HARD LIMIT SUPPRESSION FLAG - Set by soft reset, cleared when all limits physically release
// Allows motion commands after soft reset even if on limit (operator can jog away from limit)
extern volatile bool g_suppress_hard_limits;

// ✅ E-STOP PENDING FLAG - Set by ESTOP_Callback ISR (app.c), cleared by soft reset / main loop
extern volatile bool g_estop_pending;

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

/* Stop all motion and timers (for emergency stop, soft reset, etc.). */
void STEPPER_StopMotion(void);

/* Feed Hold: stop step pulses but keep steppers ENERGIZED (position preserved).
   Call on '!' real-time command.  Bresenham state survives — resume is exact. */
void STEPPER_PauseMotion(void);

/* Feed Resume: restart TMR4/OC1 from the frozen Bresenham state.
   Call on '~' real-time command after STEPPER_PauseMotion(). */
void STEPPER_ResumeMotion(void);

#endif /* STEPPER_H */