#ifndef STEPPER_H
#define STEPPER_H

#include <stdint.h>
#include <stdbool.h>
#include "common.h"
#include "data_structures.h"  // ✅ APP_DATA defined here
#include "plib_ocmp5.h"   //X Axis pulse
#include "plib_ocmp2.h"   //Y Axis pulse
#include "plib_ocmp3.h"   //Z Axis pulse
#include "plib_ocmp4.h"   //A Axis pulse





typedef struct {
    uint32_t x_steps, y_steps, z_steps, a_steps;
    float steps_per_mm_x, steps_per_mm_y, steps_per_mm_z;
    float steps_per_deg_a;
} StepperPosition;

void STEPPER_Initialize(APP_DATA* appData);                // ✅ Initialize with APP_DATA reference
void STEPPER_SetStepInterval(uint32_t interval);
void STEPPER_ScheduleStep(E_AXIS axis, uint32_t offset);  // Direct enum!
void STEPPER_DisableAxis(E_AXIS axis);                    // Direct enum!
void STEPPER_SetDirection(E_AXIS axis, bool forward);     // Set direction for axis
StepperPosition* STEPPER_GetPosition(void);

#endif /* STEPPER_H */