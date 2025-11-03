#ifndef STEPPER_H
#define STEPPER_H

#include <stdint.h>
#include <stdbool.h>
#include "../data_structures.h"  // ✅ E_AXIS defined here (parent directory)
#include "../config/default/peripheral/ocmp/plib_ocmp5.h"   //X Axis pulse
#include "../config/default/peripheral/ocmp/plib_ocmp2.h"   //Y Axis pulse
#include "../config/default/peripheral/ocmp/plib_ocmp3.h"   //Z Axis pulse
#include "../config/default/peripheral/ocmp/plib_ocmp4.h"   //A Axis pulse


typedef struct {
    uint32_t x_steps, y_steps, z_steps, a_steps;
    float steps_per_mm_x, steps_per_mm_y, steps_per_mm_z;
    float steps_per_deg_a;
} StepperPosition;

void STEPPER_Initialize(void);
void STEPPER_SetStepInterval(uint32_t interval);
void STEPPER_ScheduleStep(E_AXIS axis, uint32_t offset);  // Schedule pulse at absolute TMR2 time
void STEPPER_DisableAxis(E_AXIS axis);                    // Stop pulse generation
void STEPPER_DisableAll(void);                            // Emergency stop - disable all axes
StepperPosition* STEPPER_GetPosition(void);               // Get current position counters

// ✅ NEW: Direction control (called by motion controller before scheduling pulses)
void STEPPER_SetDirection(E_AXIS axis, bool forward);     // Set direction for axis


#endif /* STEPPER_H */