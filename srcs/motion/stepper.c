#include "stepper.h"
#include "common.h"
#include "settings.h"
#include "motion_utils.h"
#include "utils/utils.h"  // For AxisConfig and AXIS_xxx inline helpers
#include "definitions.h"
#include "../config/default/peripheral/tmr/plib_tmr4.h"  // TMR4 access (16-bit)
#include "../config/default/peripheral/tmr/plib_tmr5.h"  // TMR5 pulse width timer
#include "../config/default/peripheral/ocmp/plib_ocmp1.h"  // OC1 virtual axis timer
#include <stdio.h>
#include <string.h>
#include <stdlib.h>  // abs()

// ============================================================================
// ARCHITECTURE: Single OC1 Master Timer + GPIO Step Pulses + TMR5 Pulse Width
// ============================================================================
// - OC1 generates periodic interrupts at step rate (continuous pulse mode)
// - TMR4 (16-bit) provides time base, PR4 controls step rate
// - ISR runs Bresenham + sets GPIO pins HIGH for axes needing steps
// - TMR5 one-shot clears GPIO pins after pulse width (~3µs)
// - Priority: OC1=5, TMR5=4 sub=1 (TMR5 slightly lower to avoid nesting)
// - No blocking delays in ISR - hardware-driven pulse timing!
//
// ============================================================================
// MCC RECONFIGURATION NOTES (if changing OC module or GPIO pins):
// ============================================================================
// 1. OC Module (currently OC1):
//    - Change #include above: plib_ocmp1.h → plib_ocmp2.h, etc.
//    - Update all OCMP1_xxx calls → OCMP2_xxx throughout this file
//    - Update OC1R/OC1RS register names → OC2R/OC2RS
//    - Update callback registration in STEPPER_Initialize()
//    - Update ISR function name: OCP1_ISR → OCP2_ISR
//
// 2. GPIO Pins (Step/Dir/Enable per axis):
//    - Edit srcs/utils/utils.c in UTILS_InitAxisConfig() function
//    - Update wrapper function assignments for each axis
//    - GPIO abstraction handles all pin access via AXIS_xxx() inline helpers
//
// 3. PPS (Peripheral Pin Select):
//    - Reconfigure in MCC to route OC module output to desired pin
//    - Example: RPD5R = 12 routes OC1 to RD5 (see plib_gpio.c)
// ============================================================================

// Forward declarations
void OCP1_ISR(uintptr_t context);
void TMR5_PulseWidthCallback(uint32_t status, uintptr_t context);

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
static volatile int32_t error_x = 0;  // Added for array indexing
static volatile int32_t error_y = 0;
static volatile int32_t error_z = 0;
static volatile int32_t error_a = 0;
static volatile int32_t dominant_delta = 0;
static volatile E_AXIS dominant_axis = AXIS_X;  // Pre-calculated dominant axis

// Segment deltas (loaded when new segment starts)
static volatile int32_t delta_x = 0;
static volatile int32_t delta_y = 0;
static volatile int32_t delta_z = 0;
static volatile int32_t delta_a = 0;

// ✅ Pre-built arrays for ISR iteration (ZERO stack overhead - static storage)
// These replace local array creation on every ISR call (was 32 bytes/call)
static volatile int32_t* const error_ptrs[NUM_AXIS] = {&error_x, &error_y, &error_z, &error_a};
static volatile int32_t* const delta_ptrs[NUM_AXIS] = {&delta_x, &delta_y, &delta_z, &delta_a};

// Settings cache for ISR performance
static uint8_t step_pulse_invert_mask = 0;
static uint8_t direction_invert_mask = 0;
static uint8_t enable_invert = 0;

static bool steppers_enabled = false;

void STEPPER_Initialize(APP_DATA* appData) {
    // Store reference for ISR access
    app_data_ref = appData;
    
    // Load settings from flash
    CNC_Settings* settings = SETTINGS_GetCurrent();
    
    // Cache frequently used settings for ISR performance
    step_pulse_invert_mask = settings->step_pulse_invert;
    direction_invert_mask = settings->step_direction_invert;
    enable_invert = settings->step_enable_invert;
    
    // Update steps_per_mm from settings
    stepper_pos.steps_per_mm_x = settings->steps_per_mm_x;
    stepper_pos.steps_per_mm_y = settings->steps_per_mm_y;
    stepper_pos.steps_per_mm_z = settings->steps_per_mm_z;
    stepper_pos.steps_per_deg_a = settings->steps_per_mm_a;
    
    // Register TMR5 callback for step pulse width (clears GPIO after 3µs)
    TMR5_CallbackRegister(TMR5_PulseWidthCallback, (uintptr_t)NULL);
    
    // Register OC1 callback (master timer)
    OCMP1_CallbackRegister(OCP1_ISR, (uintptr_t)NULL);
    
    // ✅ TMR4 configuration - leave STOPPED until motion segment loaded
    // This prevents spurious ISR firing during idle
    TMR4_Stop();
    
    // ✅ OC1 is already configured by MCC (continuous pulse mode, TMR4 source, 16-bit)
    OCMP1_Enable();
    
    // Enable all stepper drivers
    MOTION_UTILS_EnableAllAxes(true, enable_invert);
    
    // ✅ DEBUG: Verify enable pins are set
    DEBUG_PRINT_STEPPER("[STEPPER_Init] Enable invert mask: 0x%02X\r\n", enable_invert);
    DEBUG_PRINT_STEPPER("[STEPPER_Init] EnX state: %d, EnY state: %d\r\n", 
        EnX_Get(), EnY_Get());
}

// ============================================================================
// Segment Loading (Called by Motion Module)
// ============================================================================

void STEPPER_LoadSegment(MotionSegment* segment) {
    if (segment == NULL) return;
 
    // Handles soft reset (ox18), emergency stop, or other disabling instructions.
    if(!steppers_enabled){
        STEPPER_EnableAll();
    }


    // Load deltas for Bresenham
    delta_x = segment->delta_x;
    delta_y = segment->delta_y;
    delta_z = segment->delta_z;
    delta_a = segment->delta_a;
    
    // Use pre-calculated dominant axis and delta from motion planning
    // (Calculated once in KINEMATICS_LinearMove, not recalculated here)
    dominant_axis = segment->dominant_axis;
    dominant_delta = segment->dominant_delta;
    
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
    
    // Update GPIO direction pins using array-based approach
    int32_t deltas[NUM_AXIS] = {delta_x, delta_y, delta_z, delta_a};
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        MOTION_UTILS_SetDirection(axis, (deltas[axis] >= 0), direction_invert_mask);
    }
    
    // Set initial step rate (PR4 controls OC1 period in 16-bit mode)
    uint16_t period = (uint16_t)segment->initial_rate;  // Ensure 16-bit value
    if (period > 65535) period = 65535;  // Clamp to 16-bit max
    TMR4_PeriodSet(period);
    
    // ✅ DEBUG: Print segment loading details
    DEBUG_PRINT_STEPPER("[STEPPER_Load] initial_rate=%lu, period=%u (%.2fms), steps=%ld\r\n",
        segment->initial_rate, period, (float)period / 12500.0f, segment->steps_remaining);
    
    // ✅ CRITICAL: Initialize OC1 compare registers for continuous pulse mode
    // OCxR = Rising edge (pulse starts)
    // OCxRS = Falling edge (generates ISR on falling edge)
    // MUST satisfy: PR4 ≥ OCxRS > OCxR (per datasheet Table 16-3)
    // Pulse width constants: 80 ticks for OC1R, 40 ticks for OC1RS
    OCMP1_CompareValueSet(period - 80);              // OCxR: Rising edge (pulse starts)
    OCMP1_CompareSecondaryValueSet(period - 40);     // OCxRS: Falling edge (ISR trigger)
    
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
    OCMP1_CompareValueSet(period - 80);              // OCxR: Rising edge
    OCMP1_CompareSecondaryValueSet(period - 40);     // OCxRS: Falling edge
}

bool STEPPER_IsEnabled(void)
{
    return steppers_enabled;
}

void STEPPER_EnableAll(void)
{
    if (steppers_enabled) return;

    /* Enable outputs for every axis. Adjust polarity if needed. */
    for (E_AXIS a = 0; a < NUM_AXIS; a++) {
        /* If your hardware uses active LOW enable, replace EnableClear with EnableSet. */
        AXIS_EnableSet(a);   /* or AXIS_EnableClear(a) depending on board */
    }

    steppers_enabled = true;
    DEBUG_PRINT_STEPPER("[STEPPER] Enabled all drivers\r\n");
}

void STEPPER_DisableAll(void)
{
    if (!steppers_enabled) return;

    for (E_AXIS a = 0; a < NUM_AXIS; a++) {
        AXIS_EnableClear(a); /* or AXIS_EnableSet(a) if active LOW */
    }

    steppers_enabled = false;
    DEBUG_PRINT_STEPPER("[STEPPER] Disabled all drivers\r\n");
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
    // ✅ CRITICAL DEBUG: Toggle LED to confirm ISR fires
    LED1_Toggle();
    
    // ===== DOMINANT AXIS STEP (ALWAYS) =====
    // Use pre-calculated dominant_axis (set during STEPPER_LoadSegment)
    // Eliminates 4 abs() calls + 3 comparisons per ISR (significant overhead reduction!)
    
    // Atomic GPIO step pulse - single instruction, zero overhead!
    AXIS_StepSet(dominant_axis);
    
    // Update step counter based on direction (inline, zero overhead)
    if (direction_bits & (1 << dominant_axis)) {
        AXIS_IncrementSteps(dominant_axis);
    } else {
        AXIS_DecrementSteps(dominant_axis);
    }
    
    // ===== BRESENHAM FOR SUBORDINATE AXES =====
    // Use pre-built static arrays - ZERO stack overhead (no local array creation)
    
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        // Skip dominant axis and axes with zero delta
        if (axis == dominant_axis || *delta_ptrs[axis] == 0) continue;
        
        // Bresenham error accumulation
        *error_ptrs[axis] += abs(*delta_ptrs[axis]);
        if (*error_ptrs[axis] >= dominant_delta) {
            // Atomic GPIO step pulse - single instruction!
            AXIS_StepSet(axis);
            
            // Update step counter (inline, zero overhead)
            if (direction_bits & (1 << axis)) {
                AXIS_IncrementSteps(axis);
            } else {
                AXIS_DecrementSteps(axis);
            }
            
            *error_ptrs[axis] -= dominant_delta;
        }
    }
    
    // ===== START TMR5 FOR PULSE WIDTH =====
    // TMR5 pre-configured to fire after 3µs (PR5=149, PBCLK3=50MHz, prescaler 1:1)
    // TMR5 period is FIXED at 3µs - do NOT change it!
    TMR5_Start();
    
    // ===== SCHEDULE NEXT STEP (CRITICAL!) =====
    // TMR4 rolls over at PR4, so we use RELATIVE compare values
    // This is what makes the motion continue - without this, only ONE step occurs!
    if (app_data_ref != NULL && app_data_ref->currentSegment != NULL) {
        uint32_t step_interval = app_data_ref->currentSegment->step_interval;  // Ticks between steps
        
        // ✅ CRITICAL: Set TMR4 period AND OC1 compare values for continuous pulses
        // Per datasheet 16.3.2.5: PR4 must be ≥ OC1RS > OC1R
        // OC1R fires first (rising edge), OC1RS fires second (falling edge + ISR)
        // Pulse width constants: 80 ticks for OC1R, 40 ticks for OC1RS
        TMR4_PeriodSet((uint16_t)step_interval);                            // Timer period (controls step rate)
        OCMP1_CompareValueSet((uint16_t)step_interval - 85);                // Rising edge (pulse starts)
        OCMP1_CompareSecondaryValueSet((uint16_t)step_interval - 45);       // Falling edge (ISR trigger)

        // Update step counter and signal main loop for velocity updates
        app_data_ref->currentSegment->steps_completed++;
        app_data_ref->motionPhase = MOTION_PHASE_VELOCITY;
    }
}

// ============================================================================
// TMR5 Callback - Clears Step Pins After 3µs Pulse Width
// ============================================================================

void TMR5_PulseWidthCallback(uint32_t status, uintptr_t context) {
    // ✅ DEBUG: Toggle LED2 to confirm TMR5 callback fires
    LED2_Toggle();
    
    // Stop TMR5 (one-shot mode)
    TMR5_Stop();
    
    // Clear all step pins using axis configuration (not hardcoded!)
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        AXIS_StepClear(axis);
    }
}


StepperPosition* STEPPER_GetPosition(void)
{
    static StepperPosition snap;

    // Defaults
    snap.x_steps = snap.y_steps = snap.z_steps = snap.a_steps = 0;
    snap.steps_per_mm_x = snap.steps_per_mm_y = snap.steps_per_mm_z = 1.0f;
    snap.steps_per_deg_a = 1.0f;

    // Live step counters (from axis config pointers)
    if (g_axis_config[AXIS_X].step_count) snap.x_steps = *g_axis_config[AXIS_X].step_count;
    if (g_axis_config[AXIS_Y].step_count) snap.y_steps = *g_axis_config[AXIS_Y].step_count;
    if (g_axis_config[AXIS_Z].step_count) snap.z_steps = *g_axis_config[AXIS_Z].step_count;
    if (g_axis_config[AXIS_A].step_count) snap.a_steps = *g_axis_config[AXIS_A].step_count;

    // Steps/mm (used by status/MPos→mm conversion)
    if (g_axis_config[AXIS_X].steps_per_mm) snap.steps_per_mm_x = *g_axis_config[AXIS_X].steps_per_mm;
    if (g_axis_config[AXIS_Y].steps_per_mm) snap.steps_per_mm_y = *g_axis_config[AXIS_Y].steps_per_mm;
    if (g_axis_config[AXIS_Z].steps_per_mm) snap.steps_per_mm_z = *g_axis_config[AXIS_Z].steps_per_mm;

    return &snap;
}

// End of stepper.c
