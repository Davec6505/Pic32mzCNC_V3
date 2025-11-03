#include "stepper.h"
#include "../common.h"
#include "settings.h"  // ✅ Add settings module
#include "motion_utils.h"  // ✅ GPIO abstraction layer
#include "definitions.h"

// Callback ISR handlers for abstraction.
void OCP5_ISR(uintptr_t context);  // X Axis Step Complete
void OCP2_ISR(uintptr_t context);  // Y Axis Step Complete
void OCP3_ISR(uintptr_t context);  // Z Axis Step Complete
void OCP4_ISR(uintptr_t context);  // A Axis Step Complete

// Global position tracking
static StepperPosition stepper_pos = {
    .x_steps = 0, .y_steps = 0, .z_steps = 0, .a_steps = 0,
    .steps_per_mm_x = 200.0f,  // Configure for your machine
    .steps_per_mm_y = 200.0f,
    .steps_per_mm_z = 200.0f,
    .steps_per_deg_a = 1.0f
};

static uint32_t current_step_interval = 1000;  // 10ms default
static uint32_t pulse_width = 2;               // 20µs pulse width

// ✅ Cache settings values for performance (avoid repeated flash reads)
static uint8_t step_pulse_invert_mask = 0;
static uint8_t direction_invert_mask = 0;
static uint8_t enable_invert = 0;

void STEPPER_Initialize(void) {
    // ✅ Load settings from flash (already initialized in main.c)
    GRBL_Settings* settings = SETTINGS_GetCurrent();
    
    // ✅ Cache frequently used settings for ISR performance
    step_pulse_invert_mask = settings->step_pulse_invert;
    direction_invert_mask = settings->step_direction_invert;
    enable_invert = settings->step_enable_invert;
    
    // ✅ Update pulse width from settings (microseconds → timer ticks)
    // Timer runs at 12.5MHz (80ns resolution), so 1µs = 12.5 ticks
    pulse_width = settings->step_pulse_time * 12.5f;  // Convert µs to ticks
    
    // ✅ Update steps_per_mm from settings
    stepper_pos.steps_per_mm_x = settings->steps_per_mm_x;
    stepper_pos.steps_per_mm_y = settings->steps_per_mm_y;
    stepper_pos.steps_per_mm_z = settings->steps_per_mm_z;
    stepper_pos.steps_per_deg_a = settings->steps_per_mm_a;  // Using A axis for degrees
    
    // initialize OC Callbacks
    OCMP5_CallbackRegister(OCP5_ISR, (uintptr_t)NULL);
    OCMP2_CallbackRegister(OCP2_ISR, (uintptr_t)NULL);
    OCMP3_CallbackRegister(OCP3_ISR, (uintptr_t)NULL);
    OCMP4_CallbackRegister(OCP4_ISR, (uintptr_t)NULL);

    // Timer2 centralized timer for all OC modules
    TMR2 = 0;
    TMR2_Start();

    // ✅ Enable all axes using motion_utils abstraction
    MOTION_UTILS_EnableAllAxes(true, enable_invert);

    // Disable all axes initially - set OCxR = OCxRS to prevent spurious pulses
    for(int i = 0; i < NUM_OF_AXIS; i++) {
        STEPPER_DisableAxis((E_AXIS)i);
    }
}

void STEPPER_ScheduleStep(E_AXIS axis, uint32_t offset) {
    // Validate axis before scheduling
    if (!IS_VALID_AXIS(axis)) {
        return; // Invalid axis - do nothing
    }
    
    uint32_t now = TMR2;
    uint32_t pulse_start = now + offset;  // ABSOLUTE value
    uint32_t pulse_end = pulse_start + pulse_width;  // ABSOLUTE value
    
    switch(axis) {
        case AXIS_X: // X axis
            OC5R = pulse_start;
            OC5RS = pulse_end;
            break;
        case AXIS_Y: // Y axis
            OC2R = pulse_start;
            OC2RS = pulse_end;
            break;
        case AXIS_Z: // Z axis
            OC3R = pulse_start;
            OC3RS = pulse_end;
            break;
        case AXIS_A: // A axis
            OC4R = pulse_start;
            OC4RS = pulse_end;
            break;
        case AXIS_COUNT:
        default:
            // Invalid axis - do nothing (safety)
            break;
    }
}

void STEPPER_DisableAxis(E_AXIS axis) {
    // Validate axis before disabling
    if (!IS_VALID_AXIS(axis)) {
        return; // Invalid axis - do nothing
    }
    
    // CRITICAL: Set equal values to stop pulse generation
    // This prevents spurious pulses during Bresenham cycles
    switch(axis) {
        case AXIS_X: OC5R = OC5RS; break;  // X disabled
        case AXIS_Y: OC2R = OC2RS; break;  // Y disabled
        case AXIS_Z: OC3R = OC3RS; break;  // Z disabled
        case AXIS_A: OC4R = OC4RS; break;  // A disabled
        case AXIS_COUNT:
        default:
            // Invalid axis - do nothing (safety)
            break;
    }
}

StepperPosition* STEPPER_GetPosition(void) // Fixed syntax error
{
    return &stepper_pos;
}

// ISR Callbacks - Following absolute compare mode architecture
void OCP5_ISR(uintptr_t context) {
    // X Axis Step Complete - Minimal ISR following architecture
    uint32_t now = TMR2;
    OC5R = now + current_step_interval;  // ABSOLUTE timer count for next pulse
    OC5RS = OC5R + pulse_width;          // ABSOLUTE timer count for pulse end
    
    stepper_pos.x_steps++;
}

void OCP2_ISR(uintptr_t context) {
    // Y Axis Step Complete
    uint32_t now = TMR2;
    OC2R = now + current_step_interval;  // ABSOLUTE timer count
    OC2RS = OC2R + pulse_width;          // ABSOLUTE timer count
    
    stepper_pos.y_steps++;
}

void OCP3_ISR(uintptr_t context) {
    // Z Axis Step Complete
    uint32_t now = TMR2;
    OC3R = now + current_step_interval;  // ABSOLUTE timer count
    OC3RS = OC3R + pulse_width;          // ABSOLUTE timer count
    
    stepper_pos.z_steps++;
}

void OCP4_ISR(uintptr_t context) {
    // A Axis Step Complete
    uint32_t now = TMR2;
    OC4R = now + current_step_interval;  // ABSOLUTE timer count
    OC4RS = OC4R + pulse_width;          // ABSOLUTE timer count
    
    stepper_pos.a_steps++;
}