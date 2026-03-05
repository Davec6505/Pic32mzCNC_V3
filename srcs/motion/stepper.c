// ============================================================================
// stepper.c  -  PIC32MZ hardware stepper driver (GRBL segment-buffer style)
// ============================================================================
// Architecture
// ------------
//  * OC1 generates a periodic ISR at the current step rate (TMR4 time-base).
//  * PR4 is set ONCE per segment and stays CONSTANT for all n_step steps.
//    There is NO acceleration math inside the ISR.
//  * Velocity changes live entirely in the segment buffer, built by
//    SEGMENT_PrepBuffer() in the main loop from the planner (MotionSegment) queue.
//  * ISR: runs Bresenham, fires GPIO, counts steps, loads the next
//    segment inline when the current one completes.
//  * TMR5 one-shot clears step pins after the pulse width (~3 us).
// ============================================================================

#include "stepper.h"
#include "common.h"
#include "settings.h"
#include "motion_utils.h"
#include "homing.h"
#include "utils/utils.h"
#include "utils/uart_utils.h"
#include "definitions.h"
#include "../config/default/peripheral/tmr/plib_tmr4.h"
#include "../config/default/peripheral/tmr/plib_tmr5.h"
#include "../config/default/peripheral/ocmp/plib_ocmp1.h"
#include "../config/default/peripheral/coretimer/plib_coretimer.h"
#include <string.h>
#include <stdlib.h>

// ============================================================================
// Forward declarations
// ============================================================================
//static void OCP1_ISR(uintptr_t context);
void TMR5_PulseWidthCallback(uint32_t status, uintptr_t context);

// ============================================================================
// Public alarm / hold flags  (externs declared in stepper.h)
// ============================================================================
volatile bool g_hard_limit_alarm     = false;
volatile bool g_suppress_hard_limits = false;
volatile bool g_estop_pending        = false;
volatile bool g_feed_hold_active     = false;
volatile bool g_feed_hold_pending    = false;

// ============================================================================
// Static state
// ============================================================================
static APP_DATA* app_data_ref = NULL;

// Position tracker (shared with g_axis_settings via pointer in utils.c)
static StepperPosition stepper_pos = {
    .steps        = {0, 0, 0, 0},
    .steps_per_mm = {200.0f, 200.0f, 200.0f, 200.0f}
};

// Direction bits: bit N = 1 means positive/forward on axis N
static volatile uint8_t direction_bits = 0x0F;

// Bresenham accumulators (dominant_delta-based, GRBL style: init = dominant/2)
static volatile int32_t  error[NUM_AXIS]     = {0, 0, 0, 0};
static volatile uint32_t abs_delta[NUM_AXIS] = {0, 0, 0, 0};
static volatile int32_t  dominant_delta = 0;
static volatile E_AXIS   dominant_axis  = AXIS_X;

// Settings cache
static uint8_t step_pulse_invert_mask = 0;
static uint8_t direction_invert_mask  = 0;
static uint8_t enable_invert          = 0;
static bool    steppers_enabled       = false;

// ============================================================================
// GRBL segment-buffer ISR state
// ============================================================================
static volatile StepSegment*  current_segment   = NULL;
static volatile StepperBlock* current_st_block  = NULL;
static volatile uint16_t      segment_step_count = 0;

// ============================================================================
// Dwell state (G4)
// ============================================================================
static volatile bool     dwell_active      = false;
static volatile uint32_t dwell_start_ticks = 0;
static volatile uint32_t dwell_duration    = 0;

// ============================================================================
// Internal: load Bresenham data from a StepperBlock into ISR-visible statics.
// Call from main loop ONLY (direction pin writes must not happen in ISR).
// ============================================================================
static void _init_bresenham_from_block(const StepperBlock* blk)
{
    direction_bits = blk->direction_bits;
    dominant_delta = (int32_t)blk->step_event_count;
    dominant_axis  = AXIS_X;

    for (E_AXIS ax = AXIS_X; ax < NUM_AXIS; ax++) {
        uint32_t s    = blk->steps[ax];
        abs_delta[ax] = s;
        error[ax]     = dominant_delta >> 1;  // standard Bresenham init
        if (s > blk->steps[dominant_axis]) { dominant_axis = ax; }
    }

    // Apply direction pins (GPIO write - safe in main loop only)
    for (E_AXIS ax = AXIS_X; ax < NUM_AXIS; ax++) {
        bool fwd = (direction_bits & (1u << ax)) != 0;
        MOTION_UTILS_SetDirection(ax, fwd, direction_invert_mask);
    }
}

// ============================================================================
// STEPPER_Initialize
// ============================================================================
void STEPPER_Initialize(APP_DATA* appData)
{
    app_data_ref = appData;

    CNC_Settings* s = SETTINGS_GetCurrent();
    step_pulse_invert_mask = s->step_pulse_invert;
    direction_invert_mask  = s->step_direction_invert;
    enable_invert          = s->step_enable_invert;

    for (E_AXIS ax = AXIS_X; ax < NUM_AXIS; ax++) {
        stepper_pos.steps_per_mm[ax] = s->steps_per_mm[ax];
        error[ax]     = 0;
        abs_delta[ax] = 0;
    }
    dominant_delta     = 0;
    dominant_axis      = AXIS_X;
    direction_bits     = 0x0F;

    current_segment    = NULL;
    current_st_block   = NULL;
    segment_step_count = 0;

    dwell_active       = false;
    dwell_start_ticks  = 0;
    dwell_duration     = 0;

    TMR5_CallbackRegister(TMR5_PulseWidthCallback, (uintptr_t)NULL);
    OCMP1_CallbackRegister(OCP1_ISR, (uintptr_t)NULL);

    TMR4_Stop();
    TMR5_Stop();
    OCMP1_Enable();   // OC configured by MCC in continuous-pulse mode on TMR4

    MOTION_UTILS_EnableAllAxes(true, enable_invert);
    steppers_enabled = true;

    DEBUG_PRINT_STEPPER("[STEPPER_Init] dir_inv=0x%02X enable_inv=0x%02X\r\n",
                        direction_invert_mask, enable_invert);
}

// ============================================================================
// STEPPER_LoadSegment
// ============================================================================
// Called by motion.c when the segment ring buffer has data.
// The MotionSegment* arg is kept for API compatibility but is NOT used for
// normal moves - we always pop from the GRBL StepSegment ring buffer.
// DWELL segments are the exception: they are handled directly.
// ============================================================================
void STEPPER_LoadSegment(MotionSegment* segment)
{
    if (app_data_ref == NULL) { return; }

    // ---- DWELL: handled via planner block, no StepSegment generated --------
    if (segment != NULL && segment->type == SEGMENT_TYPE_DWELL) {
        dwell_active       = true;
        dwell_start_ticks  = CORETIMER_CounterGet();
        dwell_duration     = segment->dwell_duration;
        segment->steps_remaining = 0;
        app_data_ref->motionActive = false;
        DEBUG_PRINT_STEPPER("[STEPPER_Load] Dwell start\r\n");
        return;
    }

    // ---- Motion segment: pop from ring buffer --------------------------------
    uint8_t tail = app_data_ref->segmentBufferTail;
    uint8_t head = app_data_ref->segmentBufferHead;

    if (tail == head) {
        DEBUG_PRINT_STEPPER("[STEPPER_Load] Segment buffer empty\r\n");
        return;
    }

    if (!steppers_enabled) { STEPPER_EnableAll(); }

    // Load segment and corresponding stepper block
    current_segment    = &app_data_ref->segmentBuffer[tail];
    current_st_block   = &app_data_ref->stepperBlocks[current_segment->st_block_index];
    segment_step_count = 0;

    // Initialise Bresenham + set direction pins (main-loop safe)
    _init_bresenham_from_block((const StepperBlock*)current_st_block);

    // Set constant step rate: period is FIXED for entire segment
    uint16_t period = (uint16_t)current_segment->step_interval;
    if (period < 7u) { period = 7u; }

    TMR4_PeriodSet(period);
    OCMP1_CompareValueSet(period - 5u);
    OCMP1_CompareSecondaryValueSet(period - 3u);

    if (!(OC1CON & _OC1CON_ON_MASK)) { OCMP1_Enable(); }
    if (!(T4CON  & _T4CON_ON_MASK))  { TMR4_Start();  }

    app_data_ref->motionActive = true;

    DEBUG_PRINT_STEPPER("[STEPPER_Load] n_step=%u step_interval=%lu\r\n",
        current_segment->n_step, (unsigned long)current_segment->step_interval);
}

// ============================================================================
// Query helpers
// ============================================================================
bool STEPPER_IsEnabled(void) { return steppers_enabled; }

bool STEPPER_IsDwellComplete(void)
{
    if (!dwell_active) { return true; }
    uint32_t elapsed = CORETIMER_CounterGet() - dwell_start_ticks;
    if (elapsed >= dwell_duration) { dwell_active = false; return true; }
    return false;
}

void STEPPER_ReloadSettings(void)
{
    CNC_Settings* s = SETTINGS_GetCurrent();
    step_pulse_invert_mask = s->step_pulse_invert;
    direction_invert_mask  = s->step_direction_invert;
    enable_invert          = s->step_enable_invert;
    MOTION_UTILS_EnableAllAxes(true, enable_invert);
    DEBUG_PRINT_STEPPER("[STEPPER_Reload] dir_inv=0x%02X enable_inv=0x%02X\r\n",
                        direction_invert_mask, enable_invert);
}

// ============================================================================
// Enable / Disable
// ============================================================================
void STEPPER_EnableAll(void)
{
    CNC_Settings* s = SETTINGS_GetCurrent();
    enable_invert = s->step_enable_invert;
    MOTION_UTILS_EnableAllAxes(true, enable_invert);
    steppers_enabled = true;
    DEBUG_PRINT_STEPPER("[STEPPER] Enabled\r\n");
}

void STEPPER_DisableAll(void)
{
    CNC_Settings* s = SETTINGS_GetCurrent();
    enable_invert = s->step_enable_invert;
    MOTION_UTILS_DisableAllAxes(enable_invert);
    steppers_enabled = false;
    DEBUG_PRINT_STEPPER("[STEPPER] Disabled\r\n");
}

// ============================================================================
// Emergency stop
// ============================================================================
void STEPPER_StopMotion(void)
{
    STEPPER_DisableAll();
    TMR4_Stop();
    TMR5_Stop();
    OCMP1_Disable();
    current_segment  = NULL;
    current_st_block = NULL;
    DEBUG_PRINT_STEPPER("[STEPPER] Motion stopped\r\n");
}

// ============================================================================
// Feed Hold  ('!')
// ============================================================================
void STEPPER_PauseMotion(void)
{
    if (app_data_ref == NULL || app_data_ref->motionQueueCount == 0) {
        g_feed_hold_active  = true;
        g_feed_hold_pending = false;
        T4CONCLR  = _T4CON_ON_MASK;
        T5CONCLR  = _T5CON_ON_MASK;
        OC1CONCLR = _OC1CON_ON_MASK;
        if (app_data_ref) { app_data_ref->motionActive = false; }
        DEBUG_PRINT_STEPPER("[STEPPER] Hold immediate\r\n");
        return;
    }
    g_feed_hold_pending = true;
    DEBUG_PRINT_STEPPER("[STEPPER] Hold pending\r\n");
}

void STEPPER_FinalizeHold(void)
{
    g_feed_hold_pending = false;
    g_feed_hold_active  = true;
    T4CONCLR  = _T4CON_ON_MASK;
    T5CONCLR  = _T5CON_ON_MASK;
    OC1CONCLR = _OC1CON_ON_MASK;
    if (app_data_ref) { app_data_ref->motionActive = false; }
    DEBUG_PRINT_STEPPER("[STEPPER] Hold finalised\r\n");
}

// ============================================================================
// Feed Resume  ('~')
// ============================================================================
void STEPPER_ResumeMotion(void)
{
    if (g_feed_hold_pending) {
        g_feed_hold_pending = false;
        DEBUG_PRINT_STEPPER("[STEPPER] Hold cancelled (was pending)\r\n");
        return;
    }
    g_feed_hold_active = false;
    if (app_data_ref == NULL) { return; }

    if (current_segment != NULL) {
        uint16_t period = (uint16_t)current_segment->step_interval;
        if (period < 7u) { period = 7u; }
        TMR4_PeriodSet(period);
        OCMP1_CompareValueSet(period - 5u);
        OCMP1_CompareSecondaryValueSet(period - 3u);
        OCMP1_Enable();
        TMR4_Start();
        app_data_ref->motionActive = true;
        DEBUG_PRINT_STEPPER("[STEPPER] Resumed interval=%u\r\n", period);
    }
}

// ============================================================================
// Direction
// ============================================================================
void STEPPER_SetDirection(E_AXIS axis, bool forward)
{
    if (!IS_VALID_AXIS(axis)) { return; }
    if (forward) { direction_bits |=  (1u << axis); }
    else         { direction_bits &= ~(1u << axis); }
    MOTION_UTILS_SetDirection(axis, forward, direction_invert_mask);
}

// ============================================================================
// OCP1_ISR  -  master step timer  (GRBL segment-buffer style)
// ============================================================================
// Pure Bresenham. No acceleration. Constant PR4 for entire segment.
// When n_step reached: advance ring-buffer tail, inline-load next segment
// (updating PR4 for the new velocity) or stop TMR4 if buffer drained.
// ============================================================================
static void OCP1_ISR(uintptr_t context)
{
    if (app_data_ref == NULL || current_segment == NULL) { return; }

    // ---- Dominant axis step -------------------------------------------------
    switch (dominant_axis) {
        case AXIS_X: StepX_Set(); break;
        case AXIS_Y: StepY_Set(); break;
        case AXIS_Z: StepZ_Set(); break;
        case AXIS_A: StepA_Set(); break;
        default: break;
    }

    // ---- Position counter ---------------------------------------------------
    if (direction_bits & (1u << dominant_axis)) { AXIS_IncrementSteps(dominant_axis); }
    else                                         { AXIS_DecrementSteps(dominant_axis); }

    // ---- Bresenham subordinate axes -----------------------------------------
    for (E_AXIS ax = AXIS_X; ax < NUM_AXIS; ax++) {
        if (ax == dominant_axis || abs_delta[ax] == 0u) { continue; }
        error[ax] += (int32_t)abs_delta[ax];
        if (error[ax] >= dominant_delta) {
            switch (ax) {
                case AXIS_X: StepX_Set(); break;
                case AXIS_Y: StepY_Set(); break;
                case AXIS_Z: StepZ_Set(); break;
                case AXIS_A: StepA_Set(); break;
                default: break;
            }
            if (direction_bits & (1u << ax)) { AXIS_IncrementSteps(ax); }
            else                              { AXIS_DecrementSteps(ax); }
            error[ax] -= dominant_delta;
        }
    }

    // ---- Start TMR5 one-shot (clears step pins after pulse width) -----------
    T5CONSET = _T5CON_ON_MASK;

    // ---- Count steps --------------------------------------------------------
    segment_step_count++;
    if (segment_step_count < current_segment->n_step) {
        return;  // Segment not yet finished
    }

    // ======== Segment complete ===============================================
    uint8_t new_tail = (app_data_ref->segmentBufferTail + 1u) % SEGMENT_BUFFER_SIZE;
    app_data_ref->segmentBufferTail = new_tail;
    app_data_ref->motionSegmentCompleted = true;  // signal main loop

    if (new_tail != app_data_ref->segmentBufferHead) {
        // ---- Next segment available: load inline (no latency) ---------------
        uint8_t old_block_index = current_segment->st_block_index;
        current_segment    = &app_data_ref->segmentBuffer[new_tail];
        current_st_block   = &app_data_ref->stepperBlocks[current_segment->st_block_index];
        segment_step_count = 0;

        // Only reload Bresenham state when the block actually changes.
        // Between segments of the SAME block the error[] accumulators must carry
        // over — resetting them at every segment bunches subordinate steps at the
        // start of each 1 ms slice instead of distributing them evenly.
        if (current_segment->st_block_index != old_block_index) {
            direction_bits = current_st_block->direction_bits;
            dominant_delta = (int32_t)current_st_block->step_event_count;
            dominant_axis  = AXIS_X;
            for (E_AXIS ax = AXIS_X; ax < NUM_AXIS; ax++) {
                uint32_t s    = current_st_block->steps[ax];
                abs_delta[ax] = s;
                error[ax]     = dominant_delta >> 1;
                if (s > current_st_block->steps[dominant_axis]) { dominant_axis = ax; }
            }
            // Direction GPIO update — ISR-safe atomic writes.
            for (E_AXIS ax = AXIS_X; ax < NUM_AXIS; ax++) {
                bool fwd = (direction_bits & (1u << ax)) != 0u;
                MOTION_UTILS_SetDirection(ax, fwd, direction_invert_mask);
            }
        }

        // Update step rate to new segment's constant interval
        uint16_t period = (uint16_t)current_segment->step_interval;
        if (period < 7u) { period = 7u; }
        PR4   = period;
        OC1R  = period - 5u;
        OC1RS = period - 3u;
    } else {
        // ---- Ring buffer empty: stop timer until main loop refills ----------
        current_segment  = NULL;
        current_st_block = NULL;
        T4CONCLR = _T4CON_ON_MASK;
        app_data_ref->motionActive = false;
    }
}

// ============================================================================
// TMR5 callback  -  clear step pins after pulse width
// ============================================================================
void TMR5_PulseWidthCallback(uint32_t status, uintptr_t context)
{
    T5CONCLR = _T5CON_ON_MASK;
    StepX_Clear();
    StepY_Clear();
    StepZ_Clear();
    StepA_Clear();
}

// ============================================================================
// Position accessors
// ============================================================================
StepperPosition* STEPPER_GetPosition(void)
{
    static StepperPosition snap;
    for (E_AXIS ax = AXIS_X; ax < NUM_AXIS; ax++) {
        snap.steps[ax]        = (g_axis_settings[ax].step_count)
                                    ? *g_axis_settings[ax].step_count  : 0;
        snap.steps_per_mm[ax] = (g_axis_settings[ax].steps_per_mm)
                                    ? *g_axis_settings[ax].steps_per_mm : 1.0f;
    }
    return &snap;
}

StepperPosition* STEPPER_GetPositionPointer(void)
{
    return &stepper_pos;
}

// End of stepper.c
