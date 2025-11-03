#ifndef STEPPER_H
#define STEPPER_H

#include <stdint.h>
#include <stdbool.h>
#include "../data_structures.h"  // ✅ E_AXIS defined here (parent directory)
#include "plib_ocmp5.h"   //X Axis pulse
#include "plib_ocmp2.h"   //Y Axis pulse
#include "plib_ocmp3.h"   //Z Axis pulse
#include "plib_ocmp4.h"   //A Axis pulse


typedef struct {
    uint32_t x_steps, y_steps, z_steps, a_steps;
    float steps_per_mm_x, steps_per_mm_y, steps_per_mm_z;
    float steps_per_deg_a;
} StepperPosition;

void STEPPER_Initialize(void);
void STEPPER_SetStepInterval(uint32_t interval);
void STEPPER_ScheduleStep(E_AXIS axis, uint32_t offset);  // Direct enum!
void STEPPER_DisableAxis(E_AXIS axis);                    // Direct enum!
StepperPosition* STEPPER_GetPosition(void);

// ✅ NEW: Segment loading and control (called by motion controller)
void STEPPER_LoadSegment(MotionSegment* segment);         // Load segment for ISR execution
void STEPPER_SetDominantAxis(E_AXIS axis);                // Set which axis is dominant
E_AXIS STEPPER_GetDominantAxis(void);                     // Query current dominant axis
bool STEPPER_IsSegmentComplete(void);                     // Check if current segment done
void STEPPER_ClearSegmentComplete(void);                  // Clear completion flag


#endif /* STEPPER_H */