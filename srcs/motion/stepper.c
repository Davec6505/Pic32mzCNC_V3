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

// ✅ E-STOP PENDING FLAG - Set by ESTOP_Callback ISR (app.c), cleared by soft reset / main loop
volatile bool g_estop_pending = false;

// ✅ FEED HOLD FLAG - Set by '!' real-time command, cleared by '~' resume or soft reset
// When true: MOTION_Tasks, arc generation, and event processing are gated in APP_IDLE
volatile bool g_feed_hold_active = false;

// ✅ FEED HOLD PENDING FLAG - Set by '!', cleared when motion queue drains to 0
// While pending: MOTION_Tasks keeps running to drain current segment gracefully.
// Arc generation and new event processing are gated to avoid adding segments.
// Status reports Hold:1 (decelerating / draining) until queue empties, then Hold:0.
volatile bool g_feed_hold_pending = false;

// Position tracking (incremented/decremented by ISR based on direction)
static StepperPosition stepper_pos = {
    .steps = {0, 0, 0, 0},                    // All axes start at zero
    .steps_per_mm = {200.0f, 200.0f, 200.0f, 200.0f}  // Default 200 steps/mm all axes
};

// Direction bits (set by motion loader before segment starts)
// Bit 0=X, 1=Y, 2=Z, 3=A (1=forward/positive, 0=reverse/negative)
static volatile uint8_t direction_bits = 0x0F;  // Default all forward

// ✅ ARRAY-BASED: Bresenham error accumulators (persist across ISR calls)
static volatile int32_t error[NUM_AXIS] = {0, 0, 0, 0};
static volatile int32_t dominant_delta = 0;
static volatile E_AXIS dominant_axis = AXIS_X;  // Pre-calculated dominant axis

// ✅ ARRAY-BASED: Segment deltas (loaded when new segment starts)
static volatile int32_t delta[NUM_AXIS] = {0, 0, 0, 0};
// Pre-computed absolute values — eliminates abs() calls from ISR hot path
static volatile uint32_t abs_delta[NUM_AXIS] = {0, 0, 0, 0};

// Settings cache for ISR performance
static uint8_t step_pulse_invert_mask = 0;
static uint8_t direction_invert_mask = 0;
static uint8_t enable_invert = 0;

static bool steppers_enabled = false;

// ✅ Dwell timer state (for SEGMENT_TYPE_DWELL)
static volatile bool dwell_active = false;       // true when dwell timer is running
static volatile uint32_t dwell_start_ticks = 0;  // Core timer value when dwell started
static volatile uint32_t dwell_duration = 0;     // How long to wait (core timer ticks)

void STEPPER_Initialize(APP_DATA* appData) {
    // Store reference for ISR access
    app_data_ref = appData;
    
    // Load settings from flash
    CNC_Settings* settings = SETTINGS_GetCurrent();
    
    // Cache frequently used settings for ISR performance
    step_pulse_invert_mask = settings->step_pulse_invert;
    direction_invert_mask = settings->step_direction_invert;
    enable_invert = settings->step_enable_invert;
    
    DEBUG_PRINT_STEPPER("[STEPPER_Init] dir_invert_mask=0x%02X ($3=%u), enable_invert=0x%02X ($4=%u)\r\n", 
                        direction_invert_mask, direction_invert_mask,
                        enable_invert, enable_invert);
    
    // ✅ ARRAY-BASED: Update steps_per_mm from settings (loop for scalability)
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        stepper_pos.steps_per_mm[axis] = settings->steps_per_mm[axis];
    }
    
    // ✅ Clear Bresenham state (prevents stale data after soft reset)
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        error[axis] = 0;
        delta[axis] = 0;
    }
    dominant_delta = 0;
    dominant_axis = AXIS_X;
    direction_bits = 0x0F;  // Default all forward
    
    // ✅ Clear dwell state
    dwell_active = false;
    dwell_start_ticks = 0;
    dwell_duration = 0;
    
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
    MOTION_UTILS_EnableAllAxes(true, enable_invert);
    
    steppers_enabled = true;
    
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
        dwell_active = true;
        dwell_start_ticks = CORETIMER_CounterGet();
        dwell_duration = segment->dwell_duration;
        
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
    if(!steppers_enabled){
        DEBUG_PRINT_STEPPER("[STEPPER_Load] Steppers were disabled, enabling now\r\n");
        STEPPER_EnableAll();
    } else {
        DEBUG_PRINT_STEPPER("[STEPPER_Load] Steppers already enabled\r\n");
    }

    // ✅ ARRAY-BASED: Load deltas for Bresenham (single loop!) + pre-compute abs values for ISR
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        delta[axis] = segment->delta[axis];
        abs_delta[axis] = (uint32_t)(delta[axis] < 0 ? -delta[axis] : delta[axis]);
    }
    
    // Use pre-calculated dominant axis and delta from motion planning
    // (Calculated once in KINEMATICS_LinearMove, not recalculated here)
    dominant_axis = segment->dominant_axis;
    dominant_delta = segment->dominant_delta;
    
    // ✅ ARRAY-BASED: Initialize Bresenham errors (single loop!)
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        error[axis] = dominant_delta / 2;
    }
    
    // ✅ ARRAY-BASED: Set directions (single loop!)
    direction_bits = 0;
    DEBUG_PRINT_STEPPER("[STEPPER_Load] Direction invert mask = 0x%02X\r\n", direction_invert_mask);
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        if (delta[axis] >= 0) {
            direction_bits |= (1 << axis);
        }
        bool forward = (delta[axis] >= 0);
        DEBUG_PRINT_STEPPER("[STEPPER_Load] Axis %d: delta=%ld, forward=%d, invert_bit=%d\r\n",
            axis, delta[axis], forward, (direction_invert_mask >> axis) & 0x01);
        // Update GPIO direction pin
        MOTION_UTILS_SetDirection(axis, forward, direction_invert_mask);
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
    DEBUG_PRINT_STEPPER("[STEPPER_Load] accel_count=%ld, nominal=%lu, accel_until=%lu, decel_after=%lu, final=%lu, step_iv=%lu\r\n",
        (long)segment->accel_count, segment->nominal_rate,
        segment->accelerate_until, segment->decelerate_after,
        segment->final_rate, segment->step_interval);
    
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

bool STEPPER_IsEnabled(void)
{
    return steppers_enabled;
}

bool STEPPER_IsDwellComplete(void)
{
    if (!dwell_active) return true;  // Not in dwell, so it's "complete"
    
    // Check if dwell timer has elapsed
    uint32_t now = CORETIMER_CounterGet();
    uint32_t elapsed = now - dwell_start_ticks;
    
    if (elapsed >= dwell_duration) {
        dwell_active = false;
        DEBUG_PRINT_STEPPER("[STEPPER_Dwell] Complete - elapsed %lu >= %lu ticks\r\n",
            (unsigned long)elapsed, (unsigned long)dwell_duration);
        return true;
    }
    
    return false;  // Still waiting
}

void STEPPER_ReloadSettings(void)
{
    // Reload cached settings from flash (called when $0-$5 change at runtime)
    CNC_Settings* settings = SETTINGS_GetCurrent();
    
    step_pulse_invert_mask = settings->step_pulse_invert;
    direction_invert_mask = settings->step_direction_invert;
    enable_invert = settings->step_enable_invert;
    
    // ✅ CRITICAL: Re-apply enable pins with updated inversion mask
    // Without this, cached enable_invert changes but GPIO state doesn't update
    MOTION_UTILS_EnableAllAxes(true, enable_invert);
    
    DEBUG_PRINT_STEPPER("[STEPPER_Reload] dir_invert_mask=0x%02X ($3=%u), enable_invert=0x%02X ($4=%u)\r\n", 
                        direction_invert_mask, direction_invert_mask, enable_invert, enable_invert);
}

void STEPPER_EnableAll(void)
{
    // Always re-enable drivers when called (don't skip if flag already set)
    // This ensures GPIO state matches the software flag, especially after soft reset

    // Re-read enable invert setting in case it changed via $4 command
    CNC_Settings* settings = SETTINGS_GetCurrent();
    enable_invert = settings->step_enable_invert;
    
    // Use MOTION_UTILS to properly apply inversion logic
     MOTION_UTILS_EnableAllAxes(true, enable_invert);
    
    steppers_enabled = true;
    DEBUG_PRINT_STEPPER("[STEPPER] Enabled all drivers (invert=0x%02X)\r\n", enable_invert);
}

void STEPPER_DisableAll(void)
{
    // Always disable drivers when called (don't skip if flag already clear)
    // This ensures GPIO state matches the software flag

    // Re-read enable invert setting in case it changed
    CNC_Settings* settings = SETTINGS_GetCurrent();
    enable_invert = settings->step_enable_invert;
    
    // Use MOTION_UTILS to properly apply inversion logic
    MOTION_UTILS_DisableAllAxes(enable_invert);
    
    steppers_enabled = false;
    DEBUG_PRINT_STEPPER("[STEPPER] Disabled all drivers (invert=0x%02X)\r\n", enable_invert);
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
// Feed Hold / Resume (GRBL real-time '!' and '~')
// ============================================================================

void STEPPER_PauseMotion(void)
{
    // If motion queue is already empty — go straight to fully stopped (Hold:0)
    if (app_data_ref == NULL || app_data_ref->motionQueueCount == 0) {
        g_feed_hold_active  = true;
        g_feed_hold_pending = false;
        T4CONCLR  = _T4CON_ON_MASK;
        T5CONCLR  = _T5CON_ON_MASK;
        OC1CONCLR = _OC1CON_ON_MASK;
        if (app_data_ref) app_data_ref->motionActive = false;
        DEBUG_PRINT_STEPPER("[STEPPER] Feed hold — idle, immediate stop (Hold:0)\r\n");
        return;
    }

    // Motion in progress — set pending flag only.
    // MOTION_Tasks keeps running to drain the current segment, then calls
    // STEPPER_FinalizeHold() when motionQueueCount reaches 0 (Hold:1 → Hold:0).
    g_feed_hold_pending = true;
    DEBUG_PRINT_STEPPER("[STEPPER] Feed hold pending — draining segments (Hold:1)\r\n");
}

// Called by MOTION_Tasks when motionQueueCount hits 0 while g_feed_hold_pending is set.
// Transitions from Hold:1 (draining) to Hold:0 (fully stopped).
void STEPPER_FinalizeHold(void)
{
    g_feed_hold_pending = false;
    g_feed_hold_active  = true;
    T4CONCLR  = _T4CON_ON_MASK;
    T5CONCLR  = _T5CON_ON_MASK;
    OC1CONCLR = _OC1CON_ON_MASK;
    if (app_data_ref != NULL) app_data_ref->motionActive = false;
    DEBUG_PRINT_STEPPER("[STEPPER] Feed hold finalized — queue drained, fully stopped (Hold:0)\r\n");
}

void STEPPER_ResumeMotion(void)
{
    // Cancel a pending hold (queue still running — just un-gate everything)
    if (g_feed_hold_pending) {
        g_feed_hold_pending = false;
        DEBUG_PRINT_STEPPER("[STEPPER] Feed hold cancelled (was pending, motion continues)\r\n");
        return;
    }

    // Full resume from parked state (Hold:0)
    g_feed_hold_active = false;

    // Guard: nothing to resume if no active segment
    // (Hold fired BETWEEN segments — MOTION_Tasks will pick up next queued segment naturally)
    if (app_data_ref == NULL || app_data_ref->currentSegment == NULL) {
        if (app_data_ref != NULL) {
            app_data_ref->motionActive = (app_data_ref->motionQueueCount > 0);
        }
        DEBUG_PRINT_STEPPER("[STEPPER] Resume: no active segment — MOTION_Tasks will load next\r\n");
        return;
    }

    // Restore the step interval that was active when hold fired.
    // seg->step_interval is updated each ISR step by the velocity profiler,
    // so it holds the exact rate at the moment of pause — perfect restart point.
    uint16_t period = (uint16_t)app_data_ref->currentSegment->step_interval;
    const uint16_t MIN_PERIOD = 7;
    if (period < MIN_PERIOD) period = MIN_PERIOD;

    // Re-apply OC1 compare registers and period (same formula as STEPPER_LoadSegment)
    TMR4_PeriodSet(period);
    OCMP1_CompareValueSet(period - 5);           // OCxR: rising edge
    OCMP1_CompareSecondaryValueSet(period - 3);  // OCxRS: falling edge ISR trigger

    // Re-enable OC1 before starting TMR4 (per datasheet requirement)
    OCMP1_Enable();

    // Restart TMR4 — ISR fires again and continues Bresenham from frozen state
    TMR4_Start();

    app_data_ref->motionActive = true;

    DEBUG_PRINT_STEPPER("[STEPPER] Feed resume — TMR4/OC1 restarted at interval=%u\r\n", period);
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
    // ✅ GUARD: cache segment pointer once — eliminates repeated volatile double-dereference.
    if (app_data_ref == NULL) return;
    MotionSegment* seg = app_data_ref->currentSegment;
    if (seg == NULL) return;

    // ===== DOMINANT AXIS STEP =====
    // switch → compiler emits jump table; each case is the Harmony #define macro which
    // expands to a single LATxSET register write. No function call, no pointer indirection.
    // GPIO pin assignments live in plib_gpio.h (MCC-managed) — change pins in MCC only.
    switch (dominant_axis) {
        case AXIS_X: StepX_Set(); break;
        case AXIS_Y: StepY_Set(); break;
        case AXIS_Z: StepZ_Set(); break;
        case AXIS_A: StepA_Set(); break;
        default:     break;  // should never reach here
    }

    DEBUG_EXEC_STEPPER({
        static uint32_t debug_counter = 0;
        if (++debug_counter >= 100) debug_counter = 0;
    });

    // Update position counter (always_inline, address never taken — genuinely inlines at -O1)
    if (direction_bits & (1U << dominant_axis)) {
        AXIS_IncrementSteps(dominant_axis);
    } else {
        AXIS_DecrementSteps(dominant_axis);
    }

    // ===== BRESENHAM SUBORDINATE AXES =====
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        if (axis == dominant_axis || delta[axis] == 0) continue;

        error[axis] += abs_delta[axis];
        if (error[axis] >= dominant_delta) {
            // Same switch pattern: Harmony macro → single LATxSET, no call overhead.
            switch (axis) {
                case AXIS_X: StepX_Set(); break;
                case AXIS_Y: StepY_Set(); break;
                case AXIS_Z: StepZ_Set(); break;
                case AXIS_A: StepA_Set(); break;
                default:     break;  // should never reach here
            }
            if (direction_bits & (1U << axis)) {
                AXIS_IncrementSteps(axis);
            } else {
                AXIS_DecrementSteps(axis);
            }
            error[axis] -= dominant_delta;
        }
    }

    // ===== PULSE WIDTH TIMER =====
    // Direct SFR — OC/TMR configuration never changes board-to-board.
    T5CONSET = _T5CON_ON_MASK;

    // ===== VELOCITY PROFILING (Taylor series — Austin/Aryzen algorithm) =====
    // Recurrence: c_n = c_{n-1} - (2·c_{n-1} + rest) / (4·n + 1)
    // This is the exact discretisation of constant linear acceleration:
    // unlike a fixed linear delta, it is physically correct in real distance,
    // eliminating the "creep then jolt" artefact at start and end of moves.
    //
    // Decel uses the same formula with a negative accel_count:
    //   (4·n + 1) becomes negative → delta is negative → step_interval INCREASES.
    // The rest term carries the integer remainder for sub-tick precision.
    uint32_t sc = seg->steps_completed;

    // Decel phase entry: load symmetric starting index, reset rest, no speed snap.
    // step_interval stays at cruise — rest accumulator builds sub-tick precision from step 1.
    if (sc == seg->decelerate_after && sc < seg->steps_remaining) {
        seg->accel_count = seg->accel_count_decel;  // = -(n_entry + accel_steps)
        seg->rest        = 0;
        seg->jerk_count  = 0;
    }

    if (sc < seg->accelerate_until) {
        // Acceleration: accel_count counts up from entry-velocity-corrected index
        if (seg->jerk_steps > 0U && seg->jerk_count < (int32_t)seg->jerk_steps) {
            // =====================================================================
            // S-CURVE JERK RAMP (accel entry)
            // =====================================================================
            // Linear interpolation from initial_rate toward (initial_rate - ref_delta)
            // where ref_delta = what ONE full Taylor step from initial_rate would give.
            //
            // CRITICAL: accel_count and rest are NOT advanced here.
            // Their values represent the correct Taylor starting index AFTER this ramp.
            // When jerk ramp ends, Taylor picks up from the real step_interval with the
            // correct accel_count → no divergence, no frozen-motor bug.
            //
            // ref_delta = 2*initial_rate / (4*(n_entry+1)+1)
            //           = 2*initial_rate / (4*accel_count + 5)   [accel_count frozen = n_entry]
            int32_t den_first = (seg->accel_count << 2) + 5;
            if (den_first < 1) den_first = 1;
            int32_t ref_delta = ((int32_t)seg->initial_rate << 1) / den_first;
            // Linear scale: apply (jerk_count+1)/jerk_steps fraction each step
            // Division by jerk_steps using pre-stored log2 bit-shift (zero extra cost)
            int32_t applied   = (ref_delta * (seg->jerk_count + 1)) >> seg->jerk_steps_log2;
            int32_t new_iv    = (int32_t)seg->initial_rate - applied;
            if (new_iv < (int32_t)seg->nominal_rate) new_iv = (int32_t)seg->nominal_rate;
            seg->step_interval = (uint32_t)new_iv;
            seg->jerk_count++;
            // accel_count and rest intentionally NOT changed — Taylor continuity preserved
        } else {
            // Normal Taylor accel (also runs when jerk_steps=0 or after jerk ramp completes)
            seg->accel_count++;
            int32_t num    = ((int32_t)seg->step_interval << 1) + seg->rest;
            int32_t den    = (seg->accel_count << 2) + 1;
            seg->rest      = num % den;
            int32_t new_iv = (int32_t)seg->step_interval - (num / den);
            if (new_iv < (int32_t)seg->nominal_rate) new_iv = (int32_t)seg->nominal_rate;
            seg->step_interval = (uint32_t)new_iv;
        }
    } else if (sc >= seg->decelerate_after) {
        // Deceleration: symmetric mirror of accel.
        // accel_count runs from -(n_entry+accel_steps) toward 0.
        // abs_n = -accel_count is always positive (and shrinking), so den is always positive.
        // Small delta at large abs_n is fine — rest accumulates sub-tick precision each step,
        // giving a smooth ramp that naturally builds into stronger braking near the end.
        seg->accel_count++;
        int32_t abs_n = -seg->accel_count;
        if (abs_n > 0) {
            int32_t den    = (abs_n << 2) + 1;                         // always positive
            int32_t num    = ((int32_t)seg->step_interval << 1) + seg->rest;
            seg->rest      = num % den;
            int32_t new_iv = (int32_t)seg->step_interval + (num / den); // ADD: interval grows = slower
            if (new_iv > (int32_t)seg->final_rate) new_iv = (int32_t)seg->final_rate;
            seg->step_interval = (uint32_t)new_iv;
        } else {
            // accel_count reached 0 — hold at final_rate
            seg->rest          = 0;
            seg->step_interval = seg->final_rate;
        }
    }
    // CRUISE (accelerate_until ≤ sc < decelerate_after): step_interval unchanged

    seg->steps_completed++;

    // ===== SEGMENT COMPLETION =====
    // Detect completion here — stops TMR4 precisely after the final step.
    // Prevents phantom ISR calls while main loop polls steps_completed.
    // STEPPER_LoadSegment() restarts TMR4 when the next segment is ready.
    if (seg->steps_completed >= seg->steps_remaining) {
        T4CONCLR = _T4CON_ON_MASK;  // Stop TMR4 — direct SFR
        app_data_ref->motionSegmentCompleted = true;
        return;
    }

    // Direct SFR writes — OC1/TMR4 are fixed silicon, never need Harmony abstraction.
    // PR4 >= OC1RS > OC1R (Output Compare datasheet 16.3.2.5)
    uint16_t period = (uint16_t)seg->step_interval;
    if (period < 7U) period = 7U;
    PR4   = period;
    OC1R  = period - 5U;
    OC1RS = period - 3U;
}

// ============================================================================
// TMR5 Callback - Clears Step Pins After 3µs Pulse Width
// ============================================================================

void TMR5_PulseWidthCallback(uint32_t status, uintptr_t context) {
    // Stop TMR5 one-shot — direct SFR, OC/TMR silicon never changes board-to-board.
    T5CONCLR = _T5CON_ON_MASK;

    // Clear all step pins via Harmony macros — MCC-managed, change pins in MCC only.
    // All four are always cleared (simpler and faster than a loop with a function pointer call).
    StepX_Clear();
    StepY_Clear();
    StepZ_Clear();
    StepA_Clear();
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
    return &stepper_pos;
}

// End of stepper.c
