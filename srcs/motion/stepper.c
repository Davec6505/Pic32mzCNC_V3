#include "stepper.h"
#include "common.h"
#include "settings.h"
#include "motion_utils.h"
#include "homing.h"  // For HomingControl and homing state checking
#include "utils/utils.h"  // For AxisConfig and AXIS_xxx inline helpers
#include "utils/uart_utils.h"  // For DEBUG_PRINT_STEPPER
#include "definitions.h"
#include "../config/default/peripheral/tmr/plib_tmr4.h"  // TMR4 access (16-bit)
#include "../config/default/peripheral/tmr/plib_tmr5.h"  // TMR5 pulse width timer
#include "../config/default/peripheral/ocmp/plib_ocmp1.h"  // OC1 virtual axis timer
#include "../config/default/peripheral/coretimer/plib_coretimer.h"  // Core timer for dwell
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

// ✅ HARD LIMIT ALARM FLAG - Set by ISR, cleared by main loop ($X command)
volatile bool g_hard_limit_alarm = false;

// ✅ HARD LIMIT SUPPRESSION FLAG - Set by soft reset, cleared when all limits physically release
// Allows motion commands after soft reset even if on limit (operator can jog away)
volatile bool g_suppress_hard_limits = false;



void STEPPER_Initialize(APP_DATA* appData) {
    // Store reference for ISR access
    app_data_ref = appData;
    
    // Load settings from flash
    CNC_Settings* settings = SETTINGS_GetCurrent();
    
    // Cache frequently used settings for ISR performance
    appData->step_pulse_invert_mask = settings->step_pulse_invert;
    appData->direction_invert_mask = settings->step_direction_invert;
    appData->enable_invert = settings->step_enable_invert;
    
    DEBUG_PRINT_STEPPER("[STEPPER_Init] dir_invert_mask=0x%02X ($3=%u)\r\n", 
                        direction_invert_mask, direction_invert_mask);
    
    // ✅ ARRAY-BASED: Update steps_per_mm from settings (loop for scalability)
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        appData->stepper_pos.steps_per_mm[axis] = settings->steps_per_mm[axis];
    }
    // ✅ Clear Bresenham state (prevents stale data after soft reset)
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        appData->bresenham_error_isr[axis] = 0;
        appData->bresenham_delta_isr[axis] = 0;
    }
    appData->bresenham_dominant_delta = 0;
    appData->bresenham_dominant_axis = AXIS_X;
    appData->direction_bits = 0x0F;  // Default all forward
    // ✅ Clear dwell state
    appData->dwell_active = false;
    appData->dwell_start_ticks = 0;
    appData->dwell_duration = 0;
    
    // Register TMR5 callback for step pulse width (clears GPIO after 3µs)
    TMR5_CallbackRegister(TMR5_PulseWidthCallback, (uintptr_t)NULL);
    
    // Register OC1 callback (master timer)
    OCMP1_CallbackRegister(OCP1_ISR, (uintptr_t)NULL);
    
    // ✅ TMR4 configuration - leave STOPPED until motion segment loaded
    // This prevents spurious ISR firing during idle
    TMR4_Stop();

    TMR5_Stop();
    // ✅ OC1 is already configured by MCC (continuous pulse mode, TMR4 source, 16-bit)
    OCMP1_Enable();
    
    // Enable all stepper drivers
    MOTION_UTILS_EnableAllAxes(true, appData->enable_invert);
    appData->steppers_enabled = true;
    
}

// ============================================================================
// Segment Loading (Called by Motion Module)
// ============================================================================

void STEPPER_LoadSegment(MotionSegment* segment) {
    if (segment == NULL) return;
 
    // ✅ DWELL SEGMENT HANDLING - No motion, just timer
    if (segment->type == SEGMENT_TYPE_DWELL) {
        DEBUG_PRINT_STEPPER("[STEPPER_Load] DWELL segment: %lu ticks (%.3f sec)\r\n",
            (unsigned long)segment->dwell_duration,
            (float)segment->dwell_duration / 100000000.0f);
        // Start dwell timer (core timer at 100MHz)
        if (app_data_ref) {
            app_data_ref->dwell_active = true;
            app_data_ref->dwell_start_ticks = CORETIMER_CounterGet();
            app_data_ref->dwell_duration = segment->dwell_duration;
        }
        // Mark segment as complete immediately (steps_remaining = 0, steps_completed = 0)
        // Motion state machine will check dwell_active in STEPPER_IsDwellComplete()
        segment->steps_remaining = 0;
        segment->steps_completed = 0;
        // Mark motion inactive during dwell (no motor motion)
        if (app_data_ref != NULL) {
            app_data_ref->motionActive = false;
        }
        return;  // ✅ DWELL doesn't start TMR4 or OC1 - just timer!
    }
 
    // ✅ MOTION SEGMENT HANDLING (LINEAR/ARC)
    // Handles soft reset (ox18), emergency stop, or other disabling instructions.
    if(app_data_ref && !app_data_ref->steppers_enabled){
        DEBUG_PRINT_STEPPER("[STEPPER_Load] Steppers were disabled, enabling now\r\n");
        STEPPER_EnableAll();
    } else {
        DEBUG_PRINT_STEPPER("[STEPPER_Load] Steppers already enabled\r\n");
    }

    // ✅ ARRAY-BASED: Load deltas for Bresenham (single loop!)
    if (app_data_ref) {
        for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
            app_data_ref->bresenham_delta_isr[axis] = segment->delta[axis];
        }
        // Use pre-calculated dominant axis and delta from motion planning
        app_data_ref->bresenham_dominant_axis = segment->dominant_axis;
        app_data_ref->bresenham_dominant_delta = segment->dominant_delta;
        // Initialize Bresenham errors
        for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
            app_data_ref->bresenham_error_isr[axis] = app_data_ref->bresenham_dominant_delta / 2;
        }
        // Set directions
        app_data_ref->direction_bits = 0;
        DEBUG_PRINT_STEPPER("[STEPPER_Load] Direction invert mask = 0x%02X\r\n", app_data_ref->direction_invert_mask);
        for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
            if (app_data_ref->bresenham_delta_isr[axis] >= 0) {
                app_data_ref->direction_bits |= (1 << axis);
            }
            bool forward = (app_data_ref->bresenham_delta_isr[axis] >= 0);
            DEBUG_PRINT_STEPPER("[STEPPER_Load] Axis %d: delta=%ld, forward=%d, invert_bit=%d\r\n",
                axis, app_data_ref->bresenham_delta_isr[axis], forward, (app_data_ref->direction_invert_mask >> axis) & 0x01);
            // Update GPIO direction pin
            MOTION_UTILS_SetDirection(axis, forward, app_data_ref->direction_invert_mask);
        }
    }
    
    // Set initial step rate (PR4 controls OC1 period in 16-bit mode)
    uint16_t period = (uint16_t)segment->initial_rate;  // Ensure 16-bit value
    if (period > 65535) period = 65535;  // Clamp to 16-bit max
    
    // ✅ CRITICAL: Ensure period is large enough for pulse width requirements
    // Need minimum 7 ticks: 5 for rising edge offset + 2 for pulse width
    const uint16_t MIN_PERIOD_FOR_PULSE = 7;
    if (period < MIN_PERIOD_FOR_PULSE) period = MIN_PERIOD_FOR_PULSE;
    
    TMR4_PeriodSet(period);
    
    // ✅ DEBUG: Print segment loading details
    // Timer frequency from MCC configuration: TMR4_FrequencyGet() returns actual Hz
    DEBUG_PRINT_STEPPER("[STEPPER_Load] TMR4_Freq=%luHz, initial_rate=%lu, period=%u (%.1fµs), steps=%ld\r\n",
        TMR4_FrequencyGet(), segment->initial_rate, period, 
        (float)period * 1000000.0f / TMR4_FrequencyGet(), segment->steps_remaining);
    
    // ✅ CRITICAL: Initialize OC1 compare registers for continuous pulse mode
    // OCxR = Rising edge (pulse starts)
    // OCxRS = Falling edge (generates ISR on falling edge)
    // MUST satisfy: PR4 ≥ OCxRS > OCxR (per datasheet Table 16-3)
    // Pulse width: 2.5µs = 2 ticks @ 781.25kHz (1:64 prescaler)
    // Period is guaranteed ≥ 7 ticks, so these values are always valid
    
    OCMP1_CompareValueSet(period - 5);                             // OCxR: Rising edge (pulse starts)
    OCMP1_CompareSecondaryValueSet(period - 3);                   // OCxRS: Falling edge (ISR trigger)
    
    // ✅ CRITICAL: Enable OC1 AFTER setting compare registers (per datasheet)
    // OC module must be configured before enabling to properly trigger ISR
    // This is essential for motion restart after soft reset or STEPPER_StopMotion()
    if(!(OC1CON & _OC1CON_ON_MASK)) {
        OCMP1_Enable();
        DEBUG_PRINT_STEPPER("[STEPPER_Load] OC1 enabled for motion\r\n");
    } else {
        DEBUG_PRINT_STEPPER("[STEPPER_Load] OC1 already enabled\r\n");
    }
    
    // ✅ START TMR4 to begin motion (only if not already running)
    // Check T4CON ON bit - if TMR4 already running from previous segment, don't restart!
    // TMR4 will keep running across multiple segments for smooth motion
    // It only stops when ALL motion is complete (queue empty in MOTION_Tasks)
    // This also handles the case where TMR4 was stopped by soft reset or emergency stop
    if(!(T4CON & _T4CON_ON_MASK)) {
        TMR4_Start();
        DEBUG_PRINT_STEPPER("[STEPPER_Load] TMR4 started for motion\r\n");
    } else {
        DEBUG_PRINT_STEPPER("[STEPPER_Load] TMR4 already running\r\n");
    }

    // Mark motion active when timer/OC are running
    if (app_data_ref != NULL) {
        app_data_ref->motionActive = true;
    }
}

// ============================================================================
// Velocity Update (Called by Motion Module During Segment Execution)
// ============================================================================

void STEPPER_SetStepRate(uint32_t rate_ticks) {
    // Enforce minimum rate to prevent TMR4 issues (16-bit mode)
    const uint16_t MIN_RATE = 7;  // Minimum for pulse width requirements (5 + 2 ticks)
    uint16_t period = (uint16_t)rate_ticks;
    if (period < MIN_RATE) period = MIN_RATE;
    if (period > 65535) period = 65535;  // Clamp to 16-bit max
    
    // ✅ CRITICAL: Update segment's step_interval so ISR uses new rate
    if (app_data_ref != NULL && app_data_ref->currentSegment != NULL) {
        app_data_ref->currentSegment->step_interval = period;
    }
    
    // Update PR4 - this changes the OC1 period
    TMR4_PeriodSet(period);
    
    // ✅ CRITICAL: Update OC1R/OC1RS to maintain valid compare relationship
    // OCxR near end of period, OCxRS = OCxR + pulse_width
    // This ensures OCxRS < PR4 so compare fires before TMR4 rollover
    // Pulse width: 2.5µs = 2 ticks @ 781.25kHz (1:64 prescaler)
    // Period is guaranteed ≥ 7 ticks, so these values are always valid
    
    OCMP1_CompareValueSet(period - 5);                             // OCxR: Rising edge
    OCMP1_CompareSecondaryValueSet(period - 3);                   // OCxRS: falling edge
}

bool STEPPER_IsEnabled(void)
{
    return (app_data_ref != NULL) ? app_data_ref->steppers_enabled : false;
}

bool STEPPER_IsDwellComplete(void)
{
    if (app_data_ref == NULL || !app_data_ref->dwell_active) return true;  // Not in dwell, so it's "complete"
    // Check if dwell timer has elapsed
    uint32_t now = CORETIMER_CounterGet();
    uint32_t elapsed = now - app_data_ref->dwell_start_ticks;
    if (elapsed >= app_data_ref->dwell_duration) {
        app_data_ref->dwell_active = false;
        DEBUG_PRINT_STEPPER("[STEPPER_Dwell] Complete - elapsed %lu >= %lu ticks\r\n",
            (unsigned long)elapsed, (unsigned long)app_data_ref->dwell_duration);
        return true;
    }
    return false;  // Still waiting
}

void STEPPER_ReloadSettings(void)
{
    // Reload cached settings from flash (called when $0-$5 change at runtime)
    CNC_Settings* settings = SETTINGS_GetCurrent();
    
    if (app_data_ref) {
        app_data_ref->step_pulse_invert_mask = settings->step_pulse_invert;
        app_data_ref->direction_invert_mask = settings->step_direction_invert;
        app_data_ref->enable_invert = settings->step_enable_invert;
        DEBUG_PRINT_STEPPER("[STEPPER_Reload] dir_invert_mask=0x%02X ($3=%u)\r\n", 
                            app_data_ref->direction_invert_mask, app_data_ref->direction_invert_mask);
    }
}

void STEPPER_EnableAll(void)
{
    // Always re-enable drivers when called (don't skip if flag already set)
    // This ensures GPIO state matches the software flag, especially after soft reset

    // Re-read enable invert setting in case it changed via $4 command
    CNC_Settings* settings = SETTINGS_GetCurrent();
    if (app_data_ref) {
        app_data_ref->enable_invert = settings->step_enable_invert;
        // Use MOTION_UTILS to properly apply inversion logic
        MOTION_UTILS_EnableAllAxes(true, app_data_ref->enable_invert);
        app_data_ref->steppers_enabled = true;
        DEBUG_PRINT_STEPPER("[STEPPER] Enabled all drivers (invert=0x%02X)\r\n", app_data_ref->enable_invert);
    }
}

void STEPPER_DisableAll(void)
{
    // Always disable drivers when called (don't skip if flag already clear)
    // This ensures GPIO state matches the software flag

    // Re-read enable invert setting in case it changed
    CNC_Settings* settings = SETTINGS_GetCurrent();
    if (app_data_ref) {
        app_data_ref->enable_invert = settings->step_enable_invert;
        // Use MOTION_UTILS to properly apply inversion logic
        MOTION_UTILS_DisableAllAxes(app_data_ref->enable_invert);
        app_data_ref->steppers_enabled = false;
        DEBUG_PRINT_STEPPER("[STEPPER] Disabled all drivers (invert=0x%02X)\r\n", app_data_ref->enable_invert);
    }
}

// ============================================================================
// Centralized Motion Stop (Emergency Stop, Soft Reset, Alarms)
// ============================================================================

void STEPPER_StopMotion(void)
{
    // Disable stepper drivers immediately (no movement)
    STEPPER_DisableAll();
    // Stop TMR4 timer (halts ISR execution)
    TMR4_Stop();
    // Stop TMR5 timer (in case pulse width timer was running)
    TMR5_Stop();
    // Disable OC1 module (stops pulse generation)
    OCMP1_Disable();
    DEBUG_PRINT_STEPPER("[STEPPER] Motion stopped (TMR4/OC1 disabled)\r\n");
}

// ============================================================================
// Direction Control (Called by Motion Module)
// ============================================================================

void STEPPER_SetDirection(E_AXIS axis, bool forward) {
    if (!IS_VALID_AXIS(axis)) {
        return;  // Invalid axis
    }
    
    // Update direction bits for ISR
    if (app_data_ref) {
        if (forward) {
            app_data_ref->direction_bits |= (1 << axis);   // Set bit (forward/positive)
        } else {
            app_data_ref->direction_bits &= ~(1 << axis);  // Clear bit (reverse/negative)
        }
        // Set GPIO direction pin using motion_utils (handles inversion from settings)
        MOTION_UTILS_SetDirection(axis, forward, app_data_ref->direction_invert_mask);
    }
}

// ============================================================================
// OC1 ISR - Master Timer (Bresenham + GPIO Step Pulses)
// ============================================================================

void OCP1_ISR(uintptr_t context) {
    // ✅ GUARD: No active segment - skip step generation but keep timer running
    if (app_data_ref == NULL || app_data_ref->currentSegment == NULL) {
        return;  // Main loop will load next segment or stop timer
    }
    
    // ===== DOMINANT AXIS STEP (ALWAYS) =====
    // Use pre-calculated dominant_axis (set during STEPPER_LoadSegment)
    if (!app_data_ref) return;
    E_AXIS dom_axis = app_data_ref->bresenham_dominant_axis;
    int32_t dom_delta = app_data_ref->bresenham_dominant_delta;
    // Atomic GPIO step pulse - single instruction, zero overhead!
    AXIS_StepSet(dom_axis);
    // Update step counter based on direction (inline, zero overhead)
    if (app_data_ref->direction_bits & (1 << dom_axis)) {
        AXIS_IncrementSteps(dom_axis);
    } else {
        AXIS_DecrementSteps(dom_axis);
    }
    // ===== BRESENHAM FOR SUBORDINATE AXES =====
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        // Skip dominant axis and axes with zero delta
        if (axis == dom_axis || app_data_ref->bresenham_delta_isr[axis] == 0) continue;
        // Bresenham error accumulation using direct array access
        app_data_ref->bresenham_error_isr[axis] += abs(app_data_ref->bresenham_delta_isr[axis]);
        if (app_data_ref->bresenham_error_isr[axis] >= dom_delta) {
            // Atomic GPIO step pulse - single instruction!
            AXIS_StepSet(axis);
            // Update step counter (inline, zero overhead)
            if (app_data_ref->direction_bits & (1 << axis)) {
                AXIS_IncrementSteps(axis);
            } else {
                AXIS_DecrementSteps(axis);
            }
            app_data_ref->bresenham_error_isr[axis] -= dom_delta;
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
        // ✅ CRITICAL: Ensure period is large enough for pulse width requirements
        const uint16_t MIN_PERIOD_FOR_PULSE = 7;  // 5 for rising edge offset + 2 for pulse width
        uint16_t period = (uint16_t)step_interval;
        if (period < MIN_PERIOD_FOR_PULSE) period = MIN_PERIOD_FOR_PULSE;
        // ✅ CRITICAL: Set TMR4 period AND OC1 compare values for continuous pulses
        // Per datasheet 16.3.2.5: PR4 must be ≥ OC1RS > OC1R
        // OC1R fires first (rising edge), OC1RS fires second (falling edge + ISR)
        // Pulse width: 2.5µs = 2 ticks @ 781.25kHz (1:64 prescaler)
        TMR4_PeriodSet(period);                                             // Timer period (controls step rate)
        OCMP1_CompareValueSet(period - 5);                                  // Rising edge (pulse starts)
        OCMP1_CompareSecondaryValueSet(period - 3);                         // Falling edge (ISR trigger)
        // Update step counter and signal main loop for velocity updates
        app_data_ref->currentSegment->steps_completed++;
        app_data_ref->motionPhase = MOTION_PHASE_VELOCITY;
    }
}

// ============================================================================
// TMR5 Callback - Clears Step Pins After 3µs Pulse Width
// ============================================================================

void TMR5_PulseWidthCallback(uint32_t status, uintptr_t context) {
    // LED2 removed - reserved for state indicator (homing/alarm only)
    
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

    // ✅ ARRAY-BASED: Zero all arrays (single loop!)
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        snap.steps[axis] = 0;
        snap.steps_per_mm[axis] = 1.0f;
    }

    // ✅ ARRAY-BASED: Copy live step counters (single loop!)
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        if (g_axis_settings[axis].step_count) {
            snap.steps[axis] = *g_axis_settings[axis].step_count;
        }
        if (g_axis_settings[axis].steps_per_mm) {
            snap.steps_per_mm[axis] = *g_axis_settings[axis].steps_per_mm;
        }
    }

    return &snap;
}

// Get pointer to LIVE position counters (for g_axis_settings initialization)
StepperPosition* STEPPER_GetPositionPointer(void)
{
    return (app_data_ref ? (StepperPosition*)&app_data_ref->stepper_pos : NULL);
}

// End of stepper.c
