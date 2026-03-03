
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

// ✅ FEED HOLD FLAG - Set when motion queue drains after '!' (Hold:0 — fully stopped)
// Gated in APP_IDLE: MOTION_Tasks, arc generation, and event processing all blocked
extern volatile bool g_feed_hold_active;

// ✅ FEED HOLD PENDING FLAG - Set by '!' while motion is in progress (Hold:1 — draining)
// MOTION_Tasks still runs; arc generation and event processing gated to prevent new segments.
// Transitions to g_feed_hold_active=true when motionQueueCount hits 0 (STEPPER_FinalizeHold).
extern volatile bool g_feed_hold_pending;

void STEPPER_Initialize(APP_DATA* appData);
void STEPPER_LoadSegment(MotionSegment* segment);         // Load new segment for execution
void STEPPER_SetDirection(E_AXIS axis, bool forward);     // Set direction for axis
void STEPPER_DisableAll(void);                            // Emergency stop
StepperPosition* STEPPER_GetPosition(void);               // Get current position (snapshot)
StepperPosition* STEPPER_GetPositionPointer(void);        // Get pointer to live position counters
float STEPPER_GetCurrentFeedrateMmMin(void);              // Real-time feedrate from step_interval (mm/min)

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

/* Feed Hold: set pending flag; if idle, stops immediately. Call on '!' real-time command. */
void STEPPER_PauseMotion(void);

/* Transition from Hold:1 → Hold:0: stop hardware after queue drains. Call from MOTION_Tasks. */
void STEPPER_FinalizeHold(void);

/* Feed Resume: cancel pending hold OR restart TMR4/OC1 from frozen Bresenham state.
   Call on '~' real-time command after STEPPER_PauseMotion(). */
void STEPPER_ResumeMotion(void);

#endif /* STEPPER_H */