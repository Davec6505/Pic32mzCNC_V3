#include "stepper.h"
#include "common.h"
#include "settings.h"
#include "motion_utils.h"
#include "definitions.h"
#include "../config/default/peripheral/tmr/plib_tmr4.h"  // TMR4 access (16-bit)
#include "../config/default/peripheral/ocmp/plib_ocmp1.h"  // OC1 master timer
#include <stdio.h>
#include <string.h>
#include <stdlib.h>  // abs()

// ============================================================================
// ARCHITECTURE: Single OC1 Master Timer + GPIO Step Pulses (GRBL Pattern)
// ============================================================================
// - OC1 generates periodic interrupts at step rate (continuous pulse mode)
// - TMR4 (16-bit) provides time base, PR4 controls step rate
// - ISR runs Bresenham + sets GPIO pins HIGH for axes needing steps
// - GPIO pins cleared after pulse width delay (~5µs)
// - No multi-axis OC coordination - simple and reliable!
// ============================================================================

// Forward declaration of OC1 ISR
void OCP1_ISR(uintptr_t context);

// ============================================================================
// Static Data - Motion Control State
// ============================================================================

// Reference to application data for ISR access
static APP_DATA* app_data_ref = NULL;

// Position tracking (incremented/decremented by ISR based on direction)
static StepperPosition stepper_pos = {
    .x_steps = 0, .y_steps = 0, .z_steps = 0, .a_steps = 0,
    .steps_per_mm_x = 200.0f,
    .steps_per_mm_y = 200.0f,
    .steps_per_mm_z = 200.0f,
    .steps_per_deg_a = 1.0f
};

// Direction bits (set by motion loader before segment starts)
// Bit 0=X, 1=Y, 2=Z, 3=A (1=forward/positive, 0=reverse/negative)
static volatile uint8_t direction_bits = 0x0F;  // Default all forward

// Bresenham error accumulators (persist across ISR calls)
static volatile int32_t error_y = 0;
static volatile int32_t error_z = 0;
static volatile int32_t error_a = 0;
static volatile int32_t dominant_delta = 0;

// Segment deltas (loaded when new segment starts)
static volatile int32_t delta_x = 0;
static volatile int32_t delta_y = 0;
static volatile int32_t delta_z = 0;
static volatile int32_t delta_a = 0;

// Pulse width configuration (from settings)
static uint32_t pulse_width_cycles = 1000;  // Default ~5µs @ 200MHz CPU

// Settings cache for ISR performance
static uint8_t step_pulse_invert_mask = 0;
static uint8_t direction_invert_mask = 0;
static uint8_t enable_invert = 0;

void STEPPER_Initialize(APP_DATA* appData) {
    // Store reference for ISR access
    app_data_ref = appData;
    
    // Load settings from flash
    GRBL_Settings* settings = SETTINGS_GetCurrent();
    
    // Cache frequently used settings for ISR performance
    step_pulse_invert_mask = settings->step_pulse_invert;
    direction_invert_mask = settings->step_direction_invert;
    enable_invert = settings->step_enable_invert;
    
    // Calculate pulse width in CPU cycles for delay loop
    // settings->step_pulse_time is in microseconds
    // At 200MHz CPU: 1µs = 200 cycles, but loop overhead ~5 cycles per iteration
    pulse_width_cycles = (uint32_t)(settings->step_pulse_time * 200.0f / 5.0f);
    if (pulse_width_cycles < 100) pulse_width_cycles = 100;  // Minimum 2.5µs
    
    // Update steps_per_mm from settings
    stepper_pos.steps_per_mm_x = settings->steps_per_mm_x;
    stepper_pos.steps_per_mm_y = settings->steps_per_mm_y;
    stepper_pos.steps_per_mm_z = settings->steps_per_mm_z;
    stepper_pos.steps_per_deg_a = settings->steps_per_mm_a;
    
    // Register OC1 callback (master timer)
    OCMP1_CallbackRegister(OCP1_ISR, (uintptr_t)NULL);
    
    // ✅ TMR4 configuration - leave STOPPED until motion segment loaded
    // This prevents spurious ISR firing during idle
    TMR4_Stop();
    
    // ✅ OC1 is already configured by MCC (continuous pulse mode, TMR4 source, 16-bit)
    OCMP1_Enable();
    
    // Enable all stepper drivers
    MOTION_UTILS_EnableAllAxes(true, enable_invert);
}

// ============================================================================
// Segment Loading (Called by Motion Module)
// ============================================================================

void STEPPER_LoadSegment(MotionSegment* segment) {
    if (segment == NULL) return;
    
    // Load deltas for Bresenham
    delta_x = segment->delta_x;
    delta_y = segment->delta_y;
    delta_z = segment->delta_z;
    delta_a = segment->delta_a;
    
    // Calculate dominant delta (largest absolute value)
    dominant_delta = abs(delta_x);
    if (abs(delta_y) > dominant_delta) dominant_delta = abs(delta_y);
    if (abs(delta_z) > dominant_delta) dominant_delta = abs(delta_z);
    if (abs(delta_a) > dominant_delta) dominant_delta = abs(delta_a);
    
    // Initialize Bresenham errors
    error_y = dominant_delta / 2;
    error_z = dominant_delta / 2;
    error_a = dominant_delta / 2;
    
    // Set directions
    direction_bits = 0;
    if (delta_x >= 0) direction_bits |= (1 << AXIS_X);
    if (delta_y >= 0) direction_bits |= (1 << AXIS_Y);
    if (delta_z >= 0) direction_bits |= (1 << AXIS_Z);
    if (delta_a >= 0) direction_bits |= (1 << AXIS_A);
    
    // Update GPIO direction pins
    MOTION_UTILS_SetDirection(AXIS_X, (delta_x >= 0), direction_invert_mask);
    MOTION_UTILS_SetDirection(AXIS_Y, (delta_y >= 0), direction_invert_mask);
    MOTION_UTILS_SetDirection(AXIS_Z, (delta_z >= 0), direction_invert_mask);
    MOTION_UTILS_SetDirection(AXIS_A, (delta_a >= 0), direction_invert_mask);
    
    // Set initial step rate (PR4 controls OC1 period in 16-bit mode)
    uint16_t period = (uint16_t)segment->initial_rate;  // Ensure 16-bit value
    if (period > 65535) period = 65535;  // Clamp to 16-bit max
    TMR4_PeriodSet(period);
    
    // ✅ CRITICAL: Initialize OC1 compare registers for continuous pulse mode
    // OCxR = Rising edge (near end of period)
    // OCxRS = Falling edge (generates ISR on falling edge)
    // MUST satisfy: OCxRS < PR4 (or compare never happens before TMR4 rollover!)
    OCMP1_CompareValueSet(period - 80);              // OC1R: Rising edge
    OCMP1_CompareSecondaryValueSet(period - 40);     // OC1RS: Falling edge (ISR trigger)
    
    // ✅ START TMR4 to begin motion (only if not already running)
    // Check T4CON ON bit - if TMR4 already running from previous segment, don't restart!
    // TMR4 will keep running across multiple segments for smooth motion
    // It only stops when ALL motion is complete (queue empty in MOTION_Tasks)
    if(!(T4CON & _T4CON_ON_MASK)) {
        TMR4_Start();
    }
}

// ============================================================================
// Velocity Update (Called by Motion Module During Segment Execution)
// ============================================================================

void STEPPER_SetStepRate(uint32_t rate_ticks) {
    // Enforce minimum rate to prevent TMR4 issues (16-bit mode)
    const uint16_t MIN_RATE = 500;  // ~40µs minimum @ 12.5MHz
    uint16_t period = (uint16_t)rate_ticks;
    if (period < MIN_RATE) period = MIN_RATE;
    if (period > 65535) period = 65535;  // Clamp to 16-bit max
    
    // Update PR4 - this changes the OC1 period
    TMR4_PeriodSet(period);
    
    // ✅ CRITICAL: Update OC1R/OC1RS to maintain valid compare relationship
    // OCxR near end of period, OCxRS = OCxR + pulse_width
    // This ensures OCxRS < PR4 so compare fires before TMR4 rollover
    OCMP1_CompareValueSet(period - 80);              // OC1R: Rising edge
    OCMP1_CompareSecondaryValueSet(period - 40);     // OC1RS: Falling edge
}

void STEPPER_DisableAll(void) {
    // Emergency stop - disable OC1 and stepper drivers
    OCMP1_Disable();
    TMR4_Stop();

    // Disable stepper drivers (cut power)
    GRBL_Settings* settings = SETTINGS_GetCurrent();
    MOTION_UTILS_EnableAllAxes(false, settings->step_enable_invert);
}

StepperPosition* STEPPER_GetPosition(void)
{
    return &stepper_pos;
}

// ============================================================================
// Direction Control (Called by Motion Module)
// ============================================================================

void STEPPER_SetDirection(E_AXIS axis, bool forward) {
    if (!IS_VALID_AXIS(axis)) {
        return;  // Invalid axis
    }
    
    // Update direction bits for ISR
    if (forward) {
        direction_bits |= (1 << axis);   // Set bit (forward/positive)
    } else {
        direction_bits &= ~(1 << axis);  // Clear bit (reverse/negative)
    }
    
    // Set GPIO direction pin using motion_utils (handles inversion from settings)
    MOTION_UTILS_SetDirection(axis, forward, direction_invert_mask);
}

// ============================================================================
// OC1 ISR - Master Timer (Bresenham + GPIO Step Pulses)
// ============================================================================

void OCP1_ISR(uintptr_t context) {
    // ===== DOMINANT AXIS STEP (ALWAYS) =====
    // Determine which axis has the largest delta (dominant)
    E_AXIS dom_axis = AXIS_X;  // Default
    int32_t max_delta = abs(delta_x);
    if (abs(delta_y) > max_delta) { dom_axis = AXIS_Y; max_delta = abs(delta_y); }
    if (abs(delta_z) > max_delta) { dom_axis = AXIS_Z; max_delta = abs(delta_z); }
    if (abs(delta_a) > max_delta) { dom_axis = AXIS_A; max_delta = abs(delta_a); }
    
    // Pulse dominant axis GPIO pin
    switch(dom_axis) {
        case AXIS_X:
            LATCSET = (1 << 1);  // RC1 HIGH
            if (direction_bits & (1 << AXIS_X)) {
                stepper_pos.x_steps++;
            } else {
                stepper_pos.x_steps--;
            }
            break;
        case AXIS_Y:
            LATCSET = (1 << 2);  // RC2 HIGH
            if (direction_bits & (1 << AXIS_Y)) {
                stepper_pos.y_steps++;
            } else {
                stepper_pos.y_steps--;
            }
            break;
        case AXIS_Z:
            LATCSET = (1 << 3);  // RC3 HIGH
            if (direction_bits & (1 << AXIS_Z)) {
                stepper_pos.z_steps++;
            } else {
                stepper_pos.z_steps--;
            }
            break;
        case AXIS_A:
            LATCSET = (1 << 4);  // RC4 HIGH
            if (direction_bits & (1 << AXIS_A)) {
                stepper_pos.a_steps++;
            } else {
                stepper_pos.a_steps--;
            }
            break;
        default:
            break;
    }
    
    // ===== BRESENHAM FOR SUBORDINATE AXES =====
    
    // Y-axis (if not dominant)
    if (dom_axis != AXIS_Y && delta_y != 0) {
        error_y += abs(delta_y);
        if (error_y >= dominant_delta) {
            LATCSET = (1 << 2);  // RC2 HIGH
            if (direction_bits & (1 << AXIS_Y)) {
                stepper_pos.y_steps++;
            } else {
                stepper_pos.y_steps--;
            }
            error_y -= dominant_delta;
        }
    }
    
    // Z-axis (if not dominant)
    if (dom_axis != AXIS_Z && delta_z != 0) {
        error_z += abs(delta_z);
        if (error_z >= dominant_delta) {
            LATCSET = (1 << 3);  // RC3 HIGH
            if (direction_bits & (1 << AXIS_Z)) {
                stepper_pos.z_steps++;
            } else {
                stepper_pos.z_steps--;
            }
            error_z -= dominant_delta;
        }
    }
    
    // A-axis (if not dominant)
    if (dom_axis != AXIS_A && delta_a != 0) {
        error_a += abs(delta_a);
        if (error_a >= dominant_delta) {
            LATCSET = (1 << 4);  // RC4 HIGH
            if (direction_bits & (1 << AXIS_A)) {
                stepper_pos.a_steps++;
            } else {
                stepper_pos.a_steps--;
            }
            error_a -= dominant_delta;
        }
    }
    
    // ===== PULSE WIDTH DELAY =====
    // Manual delay for ~5µs pulse width (adjust pulse_width_cycles as needed)
    for(volatile uint32_t i = 0; i < pulse_width_cycles; i++);
    
    // ===== CLEAR ALL STEP PINS =====
    LATCCLR = (1<<1) | (1<<2) | (1<<3) | (1<<4);
    
    // ===== SIGNAL MAIN LOOP =====
    if (app_data_ref != NULL && app_data_ref->currentSegment != NULL) {
        app_data_ref->currentSegment->steps_completed++;
        app_data_ref->motionPhase = MOTION_PHASE_VELOCITY;
    }
}

// End of stepper.c