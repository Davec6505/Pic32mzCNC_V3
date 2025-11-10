#ifndef STEPPER_H
#define STEPPER_H

#include <stdint.h>
#include <stdbool.h>
#include "common.h"
#include "data_structures.h"  // APP_DATA, MotionSegment defined here

typedef struct {
    int32_t x_steps, y_steps, z_steps, a_steps;  // Signed - can go negative!
    float steps_per_mm_x, steps_per_mm_y, steps_per_mm_z;
    float steps_per_deg_a;
} StepperPosition;

// ============================================================================
// Public API
// ============================================================================

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

#endif /* STEPPER_H */