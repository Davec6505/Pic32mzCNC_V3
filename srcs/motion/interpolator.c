/*
 * interpolator.c — Fixed-rate 100 kHz DDS step interpolator
 *
 * Architecture overview
 * ──────────────────────
 * TMR4 is reconfigured from GRBL's variable-period mode to a fixed 100 kHz
 * (10 µs) servo tick.  Every tick the Tick callback:
 *   1.  Clears any active step pulses from the PREVIOUS tick.
 *   2.  Accumulates a DDS value per axis.
 *   3.  If the DDS overflows, fires one step on that axis.
 *   4.  Decrements the remaining-tick counter; stops when zero.
 *
 * DDS accumulator
 * ───────────────
 * Each axis has a 32-bit signed accumulator.  Every tick, the increment
 * (computed by LoadMove in main-loop context) is added:
 *
 *   dds_acc[axis] += dds_inc[axis]
 *
 * A step is generated when the accumulator reaches or exceeds DDS_SCALE
 * (= 2^30).  The accumulator is then reduced by DDS_SCALE.
 *
 * The DDS increment for axis at instantaneous speed v_total [mm/s]:
 *
 *   dds_inc[axis] = round( v_total × |unit_vec[axis]| × steps_per_mm[axis]
 *                          / TICK_RATE_HZ  ×  DDS_SCALE )
 *
 * Maximum step rate per axis = TICK_RATE_HZ / 2 = 50 000 steps/s.
 *
 * Phase-accurate velocity update
 * ────────────────────────────────
 * After LoadMove the ISR calls TRAJECTORY_VelocityAt(move, elapsed_time) once
 * per tick and recomputes dds_inc[] using the current instantaneous velocity.
 * To avoid float division inside the tight ISR, the per-axis step-rate scale
 * factor (steps_per_mm / TICK_RATE) is pre-calculated in LoadMove as
 * axis_scale[axis]  [DDS counts per mm/s].  Then every tick:
 *
 *   v_now = TRAJECTORY_VelocityAt(&active_move, elapsed_s)
 *   for each axis:
 *     dds_inc[axis] = (int32_t)(v_now × |unit_vec[axis]| × axis_scale[axis])
 *
 * This is one float multiply and one cast per axis per tick — well within the
 * 2000 cycle budget at 200 MHz.
 *
 * ISR budget (200 MHz, 10 µs tick = 2000 cycles):
 *   ClearStepPins          ~  20 cycles
 *   VelocityAt (fast path) ~ 120 cycles
 *   4 × DDS accumulate +   ~  80 cycles
 *     step fire + GPIO
 *   Tick countdown + stop  ~  20 cycles
 *   ──────────────────────────────────
 *   Total estimated          ~240 cycles  (12 % of budget)
 *
 * Usage
 * ──────
 * 1.  Call INTERPOLATOR_Initialize() once from APP_Initialize.
 * 2.  For each new move: call INTERPOLATOR_LoadMove(&move).
 * 3.  Poll INTERPOLATOR_MoveComplete() from APP_IDLE to dequeue next move.
 * 4.  INTERPOLATOR_Stop/Resume for feed hold.
 */

#include "motion/interpolator.h"
#include "motion/trajectory.h"
#include "motion/motion_utils.h"
#include "utils/utils.h"
#include "settings/settings.h"
#include "data_structures.h"

// Harmony PLIB for TMR4
#include "plib_tmr4.h"

#include <math.h>
#include <string.h>
#include <stddef.h>

// ─── Module configuration ─────────────────────────────────────────────────────

// Confirmed timer: PBCLK3 = 50 MHz, 1:1 prescaler.
// PR4 = (50,000,000 / 100,000) - 1 = 499
#define INTERP_PR4_VALUE   (499u)

// ─── ISR-visible state (volatile) ────────────────────────────────────────────
// All written by main-loop, read+modified by ISR.
// Access from main loop must disable interrupts or use atomic operations.

static volatile int32_t   dds_acc[NUM_AXIS];      // current accumulator
static volatile int32_t   dds_inc[NUM_AXIS];      // increment per tick
static volatile int8_t    dds_dir[NUM_AXIS];      // +1 or -1, set by LoadMove
static volatile uint32_t  ticks_remaining;        // counts down to zero
static volatile bool      interp_active  = false; // ISR is running
static volatile bool      move_complete  = false; // move finished flag
static volatile bool      feed_hold      = false; // pause without losing state

// Step-accuracy tracking: pre-computed expected steps vs steps actually fired.
// On the final tick any shortfall is force-emitted so position is exact.
static volatile int32_t   expected_steps[NUM_AXIS]; // total steps this move (signed)
static volatile int32_t   steps_fired[NUM_AXIS];    // steps fired so far (signed)

// Pre-computed per-axis scale: (steps_per_mm / TICK_RATE) × DDS_SCALE
// A float is fine here — recomputed by LoadMove, read but not written by ISR.
static float axis_scale[NUM_AXIS];

// Copy of the active SCurveMove profile for in-ISR velocity evaluation.
// Written by LoadMove (main loop, interrupts disabled), read by ISR only.
static SCurveMove active_move;

// Feed override — applied at LoadMove time, updated by SetFeedOverride which
// also re-computes dds_inc[] and restarts if active.
static float feed_override_val = 1.0f;   // 1.0 = 100 %

// ─── Internal helpers ─────────────────────────────────────────────────────────

// Recompute dds_inc[] from current active_move and feed_override_val.
// Call with interrupts disabled or before timer is running.
static void recompute_increments(float v_entry)
{
    float v_scaled = v_entry * feed_override_val;
    for (int i = 0; i < NUM_AXIS; i++) {
        float inc_f = v_scaled * fabsf(active_move.unit_vec[i]) * axis_scale[i];
        dds_inc[i] = (int32_t)(inc_f + 0.5f);
    }
}

// ─── TMR4 callback (ISR) ─────────────────────────────────────────────────────

static void INTERPOLATOR_Tick(uint32_t status, uintptr_t context)
{
    (void)status;
    (void)context;

    if (feed_hold) return;           // paused — do nothing

    // 1.  Clear step pins from previous tick (step pulse already ≥ 10 µs wide)
    for (int i = 0; i < NUM_AXIS; i++) {
        AXIS_StepClear((E_AXIS)i);
    }

    // 2.  Phase-accurate velocity: get instantaneous speed then recompute incs.
    //     ticks_remaining counts DOWN; elapsed ticks = total - remaining.
    uint32_t elapsed_ticks = active_move.t[7] * INTERPOLATOR_TICK_RATE_HZ
                             - (float)ticks_remaining + 0.5f;
    float elapsed_s = (float)elapsed_ticks / (float)INTERPOLATOR_TICK_RATE_HZ;
    float v_now = TRAJECTORY_VelocityAt(&active_move, elapsed_s) * feed_override_val;

    for (int i = 0; i < NUM_AXIS; i++) {
        float inc_f = v_now * fabsf(active_move.unit_vec[i]) * axis_scale[i];
        dds_inc[i] = (int32_t)(inc_f + 0.5f);
    }

    // 3.  DDS accumulate + step fire
    for (int i = 0; i < NUM_AXIS; i++) {
        dds_acc[i] += dds_inc[i];
        if (dds_acc[i] >= DDS_SCALE) {
            dds_acc[i] -= DDS_SCALE;
            AXIS_StepSet((E_AXIS)i);
            if (dds_dir[i] >= 0) {
                AXIS_IncrementSteps((E_AXIS)i);
                steps_fired[i]++;
            } else {
                AXIS_DecrementSteps((E_AXIS)i);
                steps_fired[i]--;
            }
        }
    }

    // 4.  Countdown
    if (ticks_remaining > 0u) {
        ticks_remaining--;
    }
    if (ticks_remaining == 1u) {
        // Penultimate tick: flush any steps the DDS missed due to v→0 rounding.
        // Step pins set here are cleared by step-1 of the NEXT (final) tick,
        // guaranteeing a full 10µs pulse width before the timer stops.
        for (int i = 0; i < NUM_AXIS; i++) {
            int32_t deficit = expected_steps[i] - steps_fired[i];
            for (int32_t s = 0; s < deficit; s++) {
                AXIS_StepSet((E_AXIS)i);
                AXIS_IncrementSteps((E_AXIS)i);
                steps_fired[i]++;
            }
            for (int32_t s = 0; s > deficit; s--) {
                AXIS_StepSet((E_AXIS)i);
                AXIS_DecrementSteps((E_AXIS)i);
                steps_fired[i]--;
            }
        }
    }
    if (ticks_remaining == 0u) {
        // Flush step pins were cleared at the top of this tick (step 1). Stop now.
        interp_active = false;
        move_complete = true;
        TMR4_Stop();
    }
}

// ─── Public API ──────────────────────────────────────────────────────────────

void INTERPOLATOR_Initialize(void)
{
    memset((void*)dds_acc, 0, sizeof(dds_acc));
    memset((void*)dds_inc, 0, sizeof(dds_inc));
    memset((void*)dds_dir, 0, sizeof(dds_dir));
    memset((void*)expected_steps, 0, sizeof(expected_steps));
    memset((void*)steps_fired, 0, sizeof(steps_fired));
    ticks_remaining = 0;
    interp_active   = false;
    move_complete   = false;
    feed_hold       = false;
    feed_override_val = 1.0f;
    memset(axis_scale, 0, sizeof(axis_scale));
    memset(&active_move, 0, sizeof(active_move));

    // Configure TMR4 for fixed 100 kHz:
    //   PBCLK3 = 50 MHz,  prescaler 1:1,  PR4 = 499
    TMR4_PeriodSet(INTERP_PR4_VALUE);
    // Register ISR callback BEFORE starting the timer
    TMR4_CallbackRegister(INTERPOLATOR_Tick, (uintptr_t)NULL);
    // Timer remains stopped until LoadMove is called
}

void INTERPOLATOR_LoadMove(const SCurveMove *move)
{
    if (move == NULL) return;

    // Stop any running move first
    if (interp_active) {
        TMR4_Stop();
        interp_active = false;
    }

    // ── Pre-compute axis scales (float, done once per move, main-loop safe) ──
    for (int i = 0; i < NUM_AXIS; i++) {
        float spm = *g_axis_settings[i].steps_per_mm;
        // scale = steps_per_mm / TICK_RATE × DDS_SCALE
        axis_scale[i] = (spm / (float)INTERPOLATOR_TICK_RATE_HZ) * (float)DDS_SCALE;
    }

    // ── Copy the move profile so the ISR can read VelocityAt without races ───
    active_move = *move;   // struct copy

    // ── Direction pins ──────────────────────────────────────────────────────
    // Apply $3 (step_direction_invert) mask so motor wiring can be corrected
    // without swapping physical wires.  dds_dir always follows the commanded
    // direction (for position counting); only the GPIO pin is inverted.
    uint8_t dir_inv = SETTINGS_GetCurrent()->step_direction_invert;
    for (int i = 0; i < NUM_AXIS; i++) {
        bool forward = (move->unit_vec[i] >= 0.0f);
        dds_dir[i] = forward ? +1 : -1;
        MOTION_UTILS_SetDirection((E_AXIS)i, forward, dir_inv);
    }

    // ── Reset accumulators ─────────────────────────────────────────────────
    for (int i = 0; i < NUM_AXIS; i++) dds_acc[i] = 0;

    // ── Pre-compute expected steps per axis (for end-of-move flush) ───────
    for (int i = 0; i < NUM_AXIS; i++) {
        float spm = *g_axis_settings[i].steps_per_mm;
        float steps_f = move->millimeters * fabsf(move->unit_vec[i]) * spm;
        expected_steps[i] = (int32_t)(steps_f + 0.5f);
        // Sign follows direction
        if (move->unit_vec[i] < 0.0f) expected_steps[i] = -expected_steps[i];
        steps_fired[i] = 0;
    }

    // ── Compute initial DDS increments from entry velocity ────────────────
    recompute_increments(move->v_entry);

    // ── Tick count for total move duration ────────────────────────────────
    float total_s = move->t[7];
    uint32_t ticks_calc = (total_s >= 1e-9f)
        ? (uint32_t)(total_s * (float)INTERPOLATOR_TICK_RATE_HZ + 0.5f) : 0u;
    if (total_s < 1e-9f) return;
    ticks_remaining = ticks_calc + 2u;
    if (ticks_remaining <= 2u) return;

    // ── Arm and start ──────────────────────────────────────────────────────
    move_complete = false;
    interp_active = true;
    feed_hold     = false;
    TMR4_Start();
}

void INTERPOLATOR_Stop(void)
{
    feed_hold = true;     // ISR will see this on next tick and do nothing
    // Optionally stop the timer to save power; re-enable on Resume
    TMR4_Stop();
    interp_active = false;
}

void INTERPOLATOR_Resume(void)
{
    if (ticks_remaining > 0u) {
        feed_hold     = false;
        interp_active = true;
        TMR4_Start();
    }
}

bool INTERPOLATOR_IsActive(void)
{
    return interp_active;
}

bool INTERPOLATOR_MoveComplete(void)
{
    if (move_complete) {
        move_complete = false;   // clear flag after reading
        return true;
    }
    return false;
}

void INTERPOLATOR_EnableAll(void)
{
    STEPPERS_Enable();
}

void INTERPOLATOR_DisableAll(void)
{
    STEPPERS_Disable();
}

void INTERPOLATOR_SetFeedOverride(float fraction)
{
    if (fraction < INTERPOLATOR_MIN_OVERRIDE) fraction = INTERPOLATOR_MIN_OVERRIDE;
    if (fraction > INTERPOLATOR_MAX_OVERRIDE) fraction = INTERPOLATOR_MAX_OVERRIDE;
    feed_override_val = fraction;
    // dds_inc[] will be updated on the next tick via VelocityAt re-computation
}

float INTERPOLATOR_GetFeedOverride(void)
{
    return feed_override_val;
}

float INTERPOLATOR_GetInstantVelocity(void)
{
    // Reconstruct elapsed time from remaining ticks
    if (!interp_active) return 0.0f;
    float total_s   = active_move.t[7];
    float remaining = (float)ticks_remaining / (float)INTERPOLATOR_TICK_RATE_HZ;
    float elapsed   = total_s - remaining;
    if (elapsed < 0.0f) elapsed = 0.0f;
    return TRAJECTORY_VelocityAt(&active_move, elapsed) * feed_override_val;
}
