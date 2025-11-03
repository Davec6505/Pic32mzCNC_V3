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

// ============================================================================
// Static Data - Shared ISR State
// ============================================================================

// Position tracking (incremented by each axis ISR)
static StepperPosition stepper_pos = {
    .x_steps = 0, .y_steps = 0, .z_steps = 0, .a_steps = 0,
    .steps_per_mm_x = 200.0f,  // Configure for your machine
    .steps_per_mm_y = 200.0f,
    .steps_per_mm_z = 200.0f,
    .steps_per_deg_a = 1.0f
};

// Current motion segment (pointer set by motion controller)
static volatile MotionSegment* current_segment = NULL;

// Dominant axis tracking (can swap on-the-fly)
static volatile E_AXIS current_dominant_axis = AXIS_X;

// Velocity profiling (updated by dominant ISR)
static volatile uint32_t current_step_interval = 1000;  // Current interval in timer ticks
static volatile uint32_t steps_completed = 0;           // Steps completed in current segment

// Pulse configuration
static uint32_t pulse_width = 2;  // 20µs pulse width (updated from settings)

// Segment completion flag (set by dominant ISR, checked by motion controller)
static volatile bool segment_complete = false;

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

// ============================================================================
// Segment Loading and Control (Motion Controller Interface)
// ============================================================================

void STEPPER_LoadSegment(MotionSegment* segment) {
    if (segment == NULL) {
        return;  // Safety check
    }
    
    // Disable all interrupts during segment loading
    __builtin_disable_interrupts();
    
    // Set segment pointer (ISR will use this)
    current_segment = segment;
    
    // Initialize velocity profile
    current_step_interval = segment->initial_rate;
    steps_completed = 0;
    segment_complete = false;
    
    // Set dominant axis
    current_dominant_axis = segment->dominant_axis;
    
    // Initialize Bresenham errors (set in MotionSegment by kinematics)
    // Note: Errors are stored in the segment structure itself
    
    // Schedule first dominant axis pulse
    uint32_t now = TMR2;
    switch (current_dominant_axis) {
        case AXIS_X:
            OC5R = now + current_step_interval;
            OC5RS = OC5R + pulse_width;
            break;
        case AXIS_Y:
            OC2R = now + current_step_interval;
            OC2RS = OC2R + pulse_width;
            break;
        case AXIS_Z:
            OC3R = now + current_step_interval;
            OC3RS = OC3R + pulse_width;
            break;
        case AXIS_A:
            OC4R = now + current_step_interval;
            OC4RS = OC4R + pulse_width;
            break;
        default:
            break;
    }
    
    // Re-enable interrupts
    __builtin_enable_interrupts();
}

void STEPPER_SetDominantAxis(E_AXIS axis) {
    if (!IS_VALID_AXIS(axis)) {
        return;  // Invalid axis
    }
    
    // Atomic update - can be called mid-motion for dynamic swapping
    __builtin_disable_interrupts();
    current_dominant_axis = axis;
    __builtin_enable_interrupts();
}

E_AXIS STEPPER_GetDominantAxis(void) {
    return current_dominant_axis;
}

bool STEPPER_IsSegmentComplete(void) {
    return segment_complete;
}

void STEPPER_ClearSegmentComplete(void) {
    segment_complete = false;
}

// ============================================================================
// ISR Callbacks - ALL Axes Enabled
// ============================================================================
// ARCHITECTURE: Dynamic Dominant Axis with On-The-Fly Swapping
// - ALL axis ISRs are enabled and count their own steps
// - Subordinate axes: Count step, exit fast (~10 cycles)
// - Dominant axis: Count + Bresenham + velocity profiling + scheduling
// - Dominant axis can SWAP mid-motion by changing current_dominant_axis
// ============================================================================

// ============================================================================
// Subordinate Axis ISRs - Simple and Fast
// ============================================================================

void OCP2_ISR(uintptr_t context) {
    // Y Axis - Count and exit unless dominant
    stepper_pos.y_steps++;
    
    if (current_dominant_axis != AXIS_Y) {
        return;  // Subordinate - exit fast (~10 cycles total)
    }
    
    // DOMINANT AXIS PATH - Y is dominant
    // (Fall through to dominant axis logic below)
    // TODO: Implement shared dominant axis logic function
}

void OCP3_ISR(uintptr_t context) {
    // Z Axis - Count and exit unless dominant
    stepper_pos.z_steps++;
    
    if (current_dominant_axis != AXIS_Z) {
        return;  // Subordinate - exit fast
    }
    
    // DOMINANT AXIS PATH - Z is dominant
    // TODO: Implement shared dominant axis logic function
}

void OCP4_ISR(uintptr_t context) {
    // A Axis - Count and exit unless dominant
    stepper_pos.a_steps++;
    
    if (current_dominant_axis != AXIS_A) {
        return;  // Subordinate - exit fast
    }
    
    // DOMINANT AXIS PATH - A is dominant
    // TODO: Implement shared dominant axis logic function
}

// ============================================================================
// Dominant Axis ISR - X Axis (OC5)
// ============================================================================

void OCP5_ISR(uintptr_t context) {
    // X Axis - Count step first
    stepper_pos.x_steps++;
    
    if (current_dominant_axis != AXIS_X) {
        return;  // Subordinate - exit fast
    }
    
    // ========================================================================
    // DOMINANT AXIS LOGIC - Runs in whichever axis is currently dominant
    // ========================================================================
    
    if (current_segment == NULL) {
        return;  // No segment loaded - shouldn't happen, but safe exit
    }
    
    // ------------------------------------------------------------------------
    // 1. Update velocity profile (trapezoidal profiling)
    // ------------------------------------------------------------------------
    if (steps_completed <= current_segment->accelerate_until) {
        // Accelerating phase - decrease interval (speed up)
        current_step_interval -= current_segment->rate_delta;
        
        // Clamp to nominal rate (don't overshoot)
        if (current_step_interval < current_segment->nominal_rate) {
            current_step_interval = current_segment->nominal_rate;
        }
        
    } else if (steps_completed > current_segment->decelerate_after) {
        // Decelerating phase - increase interval (slow down)
        current_step_interval += current_segment->rate_delta;
        
        // Clamp to final rate (don't undershoot)
        if (current_step_interval > current_segment->final_rate) {
            current_step_interval = current_segment->final_rate;
        }
    }
    // Cruise phase: current_step_interval stays constant
    
    // ------------------------------------------------------------------------
    // 2. Bresenham for subordinate axes (schedule pulses as needed)
    // ------------------------------------------------------------------------
    
    // Y Axis subordinate logic
    if (current_dominant_axis != AXIS_Y) {  // Only if Y is subordinate
        current_segment->error_y += current_segment->delta_y;
        if (current_segment->error_y >= current_segment->delta_x) {
            // Y needs a step - schedule pulse
            uint32_t now = TMR2;
            OC2R = now + 10;  // Small offset for subordinate pulse
            OC2RS = OC2R + pulse_width;
            current_segment->error_y -= current_segment->delta_x;
        } else {
            // Y doesn't need step - disable pulse generation
            OC2R = OC2RS;
        }
    }
    
    // Z Axis subordinate logic
    if (current_dominant_axis != AXIS_Z) {  // Only if Z is subordinate
        current_segment->error_z += current_segment->delta_z;
        if (current_segment->error_z >= current_segment->delta_x) {
            uint32_t now = TMR2;
            OC3R = now + 10;
            OC3RS = OC3R + pulse_width;
            current_segment->error_z -= current_segment->delta_x;
        } else {
            OC3R = OC3RS;
        }
    }
    
    // A Axis subordinate logic
    if (current_dominant_axis != AXIS_A) {  // Only if A is subordinate
        current_segment->error_a += current_segment->delta_a;
        if (current_segment->error_a >= current_segment->delta_x) {
            uint32_t now = TMR2;
            OC4R = now + 10;
            OC4RS = OC4R + pulse_width;
            current_segment->error_a -= current_segment->delta_x;
        } else {
            OC4R = OC4RS;
        }
    }
    
    // ------------------------------------------------------------------------
    // 3. Schedule next dominant axis pulse
    // ------------------------------------------------------------------------
    uint32_t now = TMR2;
    
    // Schedule based on which axis is dominant
    switch (current_dominant_axis) {
        case AXIS_X:
            OC5R = now + current_step_interval;
            OC5RS = OC5R + pulse_width;
            break;
        case AXIS_Y:
            OC2R = now + current_step_interval;
            OC2RS = OC2R + pulse_width;
            break;
        case AXIS_Z:
            OC3R = now + current_step_interval;
            OC3RS = OC3R + pulse_width;
            break;
        case AXIS_A:
            OC4R = now + current_step_interval;
            OC4RS = OC4R + pulse_width;
            break;
        default:
            break;
    }
    
    // ------------------------------------------------------------------------
    // 4. Update counters and check segment completion
    // ------------------------------------------------------------------------
    steps_completed++;
    
    if (steps_completed >= current_segment->steps_remaining) {
        // Segment complete!
        segment_complete = true;
        
        // Disable all axis pulses (set OCxR = OCxRS)
        OC5R = OC5RS;  // X
        OC2R = OC2RS;  // Y
        OC3R = OC3RS;  // Z
        OC4R = OC4RS;  // A
        
        // Motion controller will load next segment in state machine
    }
}