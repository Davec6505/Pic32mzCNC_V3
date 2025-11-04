#include "stepper.h"
#include "../common.h"
#include "settings.h"  // ✅ Add settings module
#include "motion_utils.h"  // ✅ GPIO abstraction layer
#include "definitions.h"
#include "../config/default/peripheral/tmr/plib_tmr2.h"  // ✅ TMR2 access
#include "../config/default/peripheral/ocmp/plib_ocmp5.h"  // ✅ X axis (OC5)
#include "../config/default/peripheral/ocmp/plib_ocmp1.h"  // ✅ Y axis (OC1)
#include "../config/default/peripheral/ocmp/plib_ocmp3.h"  // ✅ Z axis (OC3)
#include "../config/default/peripheral/ocmp/plib_ocmp4.h"  // ✅ A axis (OC4)

// Callback ISR handlers for abstraction.
void OCP5_ISR(uintptr_t context);  // X Axis Step Complete
void OCP1_ISR(uintptr_t context);  // Y Axis Step Complete
void OCP3_ISR(uintptr_t context);  // Z Axis Step Complete
void OCP4_ISR(uintptr_t context);  // A Axis Step Complete

// ============================================================================
// Static Data - Motion Control State
// ============================================================================

// Reference to application data for ISR phase signaling
static APP_DATA* app_data_ref = NULL;

// Position tracking (incremented/decremented by ISRs based on direction)
static StepperPosition stepper_pos = {
    .x_steps = 0, .y_steps = 0, .z_steps = 0, .a_steps = 0,
    .steps_per_mm_x = 200.0f,
    .steps_per_mm_y = 200.0f,
    .steps_per_mm_z = 200.0f,
    .steps_per_deg_a = 1.0f
};

// Direction bits (set by MOTION_Tasks before scheduling pulses)
// Bit 0=X, 1=Y, 2=Z, 3=A (1=forward/positive, 0=reverse/negative)
static volatile uint8_t direction_bits = 0x0F;  // Default all forward

// Pulse configuration
static uint32_t pulse_width = 25;  // Default 2μs @ 12.5MHz = 25 ticks

// ✅ Cache settings values for performance (avoid repeated flash reads)
static uint8_t step_pulse_invert_mask = 0;
static uint8_t direction_invert_mask = 0;
static uint8_t enable_invert = 0;

void STEPPER_Initialize(APP_DATA* appData) {
    // Store reference for ISR access
    app_data_ref = appData;
    
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
    OCMP1_CallbackRegister(OCP1_ISR, (uintptr_t)NULL);
    OCMP3_CallbackRegister(OCP3_ISR, (uintptr_t)NULL);
    OCMP4_CallbackRegister(OCP4_ISR, (uintptr_t)NULL);

    // ✅ CRITICAL FIX: Set OCxR = OCxRS to a HIGH value to prevent spurious matches
    // According to PIC32MZ datasheet Table 16-2:
    // - OCxR = 0 with TMR2 = 0 generates immediate rising edge (spurious pulse!)
    // - Setting both to 0xFFFFFFFF ensures no match until we schedule a real step
    uint32_t safe_value = 0xFFFFFFFF;  // Maximum value - won't match TMR2 during normal operation
    
    OCMP5_CompareValueSet(safe_value);
    OCMP5_CompareSecondaryValueSet(safe_value);
    OCMP1_CompareValueSet(safe_value);
    OCMP1_CompareSecondaryValueSet(safe_value);
    OCMP3_CompareValueSet(safe_value);
    OCMP3_CompareSecondaryValueSet(safe_value);
    OCMP4_CompareValueSet(safe_value);
    OCMP4_CompareSecondaryValueSet(safe_value);

    // ✅ Now safe to enable all OC modules (no spurious pulses)
    OCMP5_Enable();  // X axis
    OCMP1_Enable();  // Y axis
    OCMP3_Enable();  // Z axis
    OCMP4_Enable();  // A axis

    // Timer2 centralized timer for all OC modules
    TMR2_Stop();   // ✅ Stop timer before starting (resets to 0)
    
    // ✅ CRITICAL: Set period to maximum for free-running 32-bit timer
    // Harmony sets PR2=312499999 (~25 sec) - we want maximum for motion control
    // In 32-bit mode (T32=1), PR2 is a 32-bit register
    TMR2_PeriodSet(0xFFFFFFFF);  // Maximum ~343 seconds @ 12.5MHz
    
    TMR2_Start();  // ✅ Start with counter at 0

    // ✅ Enable all axes using motion_utils abstraction
    MOTION_UTILS_EnableAllAxes(true, enable_invert);

    // ✅ Redundant but safe - ensure all axes disabled (OCxR = OCxRS)
    for(int i = 0; i < NUM_OF_AXIS; i++) {
        STEPPER_DisableAxis((E_AXIS)i);
    }
}

void STEPPER_ScheduleStep(E_AXIS axis, uint32_t offset) {
    // Validate axis before scheduling
    if (!IS_VALID_AXIS(axis)) {
        return; // Invalid axis - do nothing
    }
    
    uint32_t now = TMR2_CounterGet();  // ✅ Use PLIB function
    
    // ✅ CRITICAL: Ensure minimum offset to prevent missed interrupts
    // If offset is too small, TMR2 might pass the compare value before OC module latches it
    if(offset < 100) offset = 100;  // Minimum 8μs safety margin @ 12.5MHz
    
    uint32_t pulse_start = now + offset;  // ABSOLUTE value
    uint32_t pulse_end = pulse_start + pulse_width;  // ABSOLUTE value
    
    switch(axis) {
        case AXIS_X: // X axis
            OCMP5_CompareValueSet(pulse_start);           // ✅ PLIB OCxR
            OCMP5_CompareSecondaryValueSet(pulse_end);    // ✅ PLIB OCxRS
            break;
        case AXIS_Y: // Y axis
            OCMP1_CompareValueSet(pulse_start);           // ✅ PLIB OCxR
            OCMP1_CompareSecondaryValueSet(pulse_end);    // ✅ PLIB OCxRS
            break;
        case AXIS_Z: // Z axis
            OCMP3_CompareValueSet(pulse_start);           // ✅ PLIB OCxR
            OCMP3_CompareSecondaryValueSet(pulse_end);    // ✅ PLIB OCxRS
            break;
        case AXIS_A: // A axis
            OCMP4_CompareValueSet(pulse_start);           // ✅ PLIB OCxR
            OCMP4_CompareSecondaryValueSet(pulse_end);    // ✅ PLIB OCxRS
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
    
    // ✅ CRITICAL: Set to safe high value to prevent spurious matches
    // Per datasheet Table 16-2: OCxR = 0 causes spurious pulses when TMR2 = 0
    uint32_t safe_value = 0xFFFFFFFF;
    
    switch(axis) {
        case AXIS_X:  // X disabled
            OCMP5_CompareValueSet(safe_value);
            OCMP5_CompareSecondaryValueSet(safe_value);
            break;
        case AXIS_Y:  // Y disabled
            OCMP1_CompareValueSet(safe_value);
            OCMP1_CompareSecondaryValueSet(safe_value);
            break;
        case AXIS_Z:  // Z disabled
            OCMP3_CompareValueSet(safe_value);
            OCMP3_CompareSecondaryValueSet(safe_value);
            break;
        case AXIS_A:  // A disabled
            OCMP4_CompareValueSet(safe_value);
            OCMP4_CompareSecondaryValueSet(safe_value);
            break;
        case AXIS_COUNT:
        default:
            // Invalid axis - do nothing (safety)
            break;
    }
}

void STEPPER_DisableAll(void) {
    // Emergency stop - disable all axes immediately
    // Set OCxR = OCxRS to safe high value to prevent spurious pulses
    uint32_t safe_value = 0xFFFFFFFF;
    
    OCMP5_CompareValueSet(safe_value);
    OCMP5_CompareSecondaryValueSet(safe_value);  // ✅ X axis
    
    OCMP2_CompareValueSet(safe_value);
    OCMP2_CompareSecondaryValueSet(safe_value);  // ✅ Y axis
    
    OCMP3_CompareValueSet(safe_value);
    OCMP3_CompareSecondaryValueSet(safe_value);  // ✅ Z axis
    
    OCMP4_CompareValueSet(safe_value);
    OCMP4_CompareSecondaryValueSet(safe_value);  // ✅ A axis
    
    // Also disable stepper enable pins (cut power to drivers)
    GRBL_Settings* settings = SETTINGS_GetCurrent();
    MOTION_UTILS_EnableAllAxes(false, settings->step_enable_invert);
}

StepperPosition* STEPPER_GetPosition(void)
{
    return &stepper_pos;
}

// ============================================================================
// Direction Control (Called by Motion Controller)
// ============================================================================

void STEPPER_SetDirection(E_AXIS axis, bool forward) {
    if (!IS_VALID_AXIS(axis)) {
        return;  // Invalid axis
    }
    
    // Update direction bits for ISRs
    if (forward) {
        direction_bits |= (1 << axis);   // Set bit (forward/positive)
    } else {
        direction_bits &= ~(1 << axis);  // Clear bit (reverse/negative)
    }
    
    // Set GPIO direction pin using motion_utils (handles inversion from settings)
    MOTION_UTILS_SetDirection(axis, forward, direction_invert_mask);
}

// ============================================================================
// ISR Callbacks - Position Counters + Phase Signaling
// ============================================================================
// ARCHITECTURE: Minimal ISRs - count steps + signal main loop when dominant axis fires
// - Position counter: increment/decrement based on direction
// - Phase signaling: if this axis is dominant, wake main loop for next phase
// - ALL motion logic runs in MOTION_Tasks() state machine (not here!)
// - ISRs execute in ~10-15 cycles (count + conditional phase set + exit)
// ============================================================================

void OCP5_ISR(uintptr_t context) {
    // ✅ DEBUG: ALWAYS increment counter for hardware test (bypass motion check)
    // X Axis - position counter  
    if (direction_bits & (1 << AXIS_X)) {
        stepper_pos.x_steps++;
    } else {
        stepper_pos.x_steps--;
    }
    
    // ✅ CRITICAL: Only process motion logic if motion is active
    if (app_data_ref != NULL && app_data_ref->currentSegment != NULL) {
        // ✅ CRITICAL: Signal main loop if X is dominant axis
        if (app_data_ref->dominantAxis == AXIS_X) {
            // Increment segment step counter (only for dominant axis)
            app_data_ref->currentSegment->steps_completed++;
            app_data_ref->motionPhase = MOTION_PHASE_VELOCITY;  // Wake main loop
        }
    }
}

void OCP1_ISR(uintptr_t context) {
    // ✅ CRITICAL: Only process if motion is active
    if (app_data_ref == NULL || app_data_ref->currentSegment == NULL) {
        return;  // No active motion - ignore spurious interrupt
    }
    
    // Y Axis - position counter + phase signaling
    if (direction_bits & (1 << AXIS_Y)) {
        stepper_pos.y_steps++;
    } else {
        stepper_pos.y_steps--;
    }
    
    // ✅ CRITICAL: Signal main loop if Y is dominant axis
    if (app_data_ref->dominantAxis == AXIS_Y) {
        // Increment segment step counter (only for dominant axis)
        app_data_ref->currentSegment->steps_completed++;
        app_data_ref->motionPhase = MOTION_PHASE_VELOCITY;  // Wake main loop
    }
}

void OCP3_ISR(uintptr_t context) {
    // ✅ CRITICAL: Only process if motion is active
    if (app_data_ref == NULL || app_data_ref->currentSegment == NULL) {
        return;  // No active motion - ignore spurious interrupt
    }
    
    // Z Axis - position counter + phase signaling
    if (direction_bits & (1 << AXIS_Z)) {
        stepper_pos.z_steps++;
    } else {
        stepper_pos.z_steps--;
    }
    
    // ✅ CRITICAL: Signal main loop if Z is dominant axis
    if (app_data_ref->dominantAxis == AXIS_Z) {
        // Increment segment step counter (only for dominant axis)
        app_data_ref->currentSegment->steps_completed++;
        app_data_ref->motionPhase = MOTION_PHASE_VELOCITY;  // Wake main loop
    }
}

void OCP4_ISR(uintptr_t context) {
    // ✅ CRITICAL: Only process if motion is active
    if (app_data_ref == NULL || app_data_ref->currentSegment == NULL) {
        return;  // No active motion - ignore spurious interrupt
    }
    
    // A Axis - position counter + phase signaling
    if (direction_bits & (1 << AXIS_A)) {
        stepper_pos.a_steps++;
    } else {
        stepper_pos.a_steps--;
    }
    
    // ✅ CRITICAL: Signal main loop if A is dominant axis
    if (app_data_ref->dominantAxis == AXIS_A) {
        // Increment segment step counter (only for dominant axis)
        app_data_ref->currentSegment->steps_completed++;
        app_data_ref->motionPhase = MOTION_PHASE_VELOCITY;  // Wake main loop
    }
}

// ============================================================================
// TMR2 Rollover Management (343.6 Second Safety)
// ============================================================================

#define TMR2_RESET_THRESHOLD  0xF0000000  // ~328 seconds, leaves 15.6s margin

static bool tmr2_reset_pending = false;

void STEPPER_CheckTimerRollover(APP_DATA* appData) {
    uint32_t tmr_now = TMR2_CounterGet();  // ✅ Use PLIB function
    
    // Check if we're approaching rollover
    if (tmr_now > TMR2_RESET_THRESHOLD && !tmr2_reset_pending) {
        tmr2_reset_pending = true;
        // Signal to stop accepting new motion (handled by caller)
    }
    
    // Only reset when motion queue is empty (safe to reset)
    if (tmr2_reset_pending && appData->motionQueueCount == 0) {
        // Disable all OC modules (stops interrupts and pulse generation)
        OCMP5_Disable();  // ✅ X axis - PLIB function
        OCMP2_Disable();  // ✅ Y axis - PLIB function
        OCMP3_Disable();  // ✅ Z axis - PLIB function
        OCMP4_Disable();  // ✅ A axis - PLIB function
        
        // Stop and restart TMR2 (resets counter to 0)
        TMR2_Stop();   // ✅ Use PLIB function
        TMR2_Start();  // ✅ Use PLIB function (counter resets to 0)
        
        // Clear all compare registers (fresh start)
        OCMP5_CompareValueSet(0);              // ✅ X axis OCxR
        OCMP5_CompareSecondaryValueSet(0);     // ✅ X axis OCxRS
        OCMP2_CompareValueSet(0);              // ✅ Y axis OCxR
        OCMP2_CompareSecondaryValueSet(0);     // ✅ Y axis OCxRS
        OCMP3_CompareValueSet(0);              // ✅ Z axis OCxR
        OCMP3_CompareSecondaryValueSet(0);     // ✅ Z axis OCxRS
        OCMP4_CompareValueSet(0);              // ✅ A axis OCxR
        OCMP4_CompareSecondaryValueSet(0);     // ✅ A axis OCxRS
        
        // Re-enable all OC modules
        OCMP5_Enable();  // ✅ X axis - PLIB function
        OCMP2_Enable();  // ✅ Y axis - PLIB function
        OCMP3_Enable();  // ✅ Z axis - PLIB function
        OCMP4_Enable();  // ✅ A axis - PLIB function
        
        tmr2_reset_pending = false;  // Resume motion
    }
}