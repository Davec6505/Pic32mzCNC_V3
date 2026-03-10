/*
 * interpolator.h — Fixed-rate 100 kHz DDS step interpolator
 *
 * Replaces the GRBL variable-period OC1/segment_buffer architecture with a
 * constant-rate servo tick driven by TMR4.
 *
 * Architecture overview:
 *   • TMR4 is reconfigured in MCC to fixed PR4=499, prescaler 1:1
 *     → 50 MHz / 500 = 100,000 Hz (10 µs per tick)
 *   • Each tick the ISR evaluates v(t) from the current SCurveMove phase
 *     table, splits it to per-axis DDS increments, and emits step pulses.
 *   • Step pulse width = 1 tick = 10 µs (valid for DRV8825 ≥1 µs, TMC5160 ≥100 ns).
 *     No separate pulse-clear timer (TMR5) is needed.
 *   • All floating-point math (profile evaluation, DDS increment computation)
 *     stays in INTERPOLATOR_LoadMove (main-loop context).
 *     The ISR itself is pure integer: 32-bit add + overflow compare per axis.
 *
 * DDS fundamentals:
 *   increment[axis] = round((v_now_mm_s * steps_per_mm[axis] / TICK_RATE) * DDS_SCALE)
 *   Each tick:  acc[axis] += increment[axis]
 *               if acc[axis] >= DDS_SCALE → emit step, acc[axis] -= DDS_SCALE
 *   This distributes steps uniformly (Bresenham-free, no step bunching).
 *
 * Feed override:
 *   Set feed_override (0.0–2.0) any time.  The next INTERPOLATOR_LoadMove
 *   or mid-move increment re-scale picks it up.  No buffer flush required.
 */

#ifndef INTERPOLATOR_H
#define INTERPOLATOR_H

#include "data_structures.h"
#include "motion/trajectory.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ─── Configuration ────────────────────────────────────────────────────────────

// Must match the TMR4 hardware: 50 MHz PBCLK3 / prescaler 1:1 / PR4=499
// Confirm in plib_tmr4.c: TMR4_FrequencyGet() must return 100000.
#define INTERPOLATOR_TICK_RATE_HZ   100000UL

// DDS accumulator scale — use full positive range of int32_t.
// Chosen as 2^30 so that (acc + inc) never wraps a signed 32-bit int
// even at maximum step rate (50 k steps/s → inc = DDS_SCALE/2 per tick).
#define DDS_SCALE   0x40000000L     // 2^30 = 1,073,741,824

// Maximum step rate per axis (Nyquist: one step per 2 ticks).
// At 100 kHz: 50,000 steps/sec.  Increase TICK_RATE if more is needed.
#define INTERPOLATOR_MAX_STEP_RATE  (INTERPOLATOR_TICK_RATE_HZ / 2u)

// Feed override range
#define INTERPOLATOR_MIN_OVERRIDE   0.1f
#define INTERPOLATOR_MAX_OVERRIDE   2.0f


// ─── Public API ───────────────────────────────────────────────────────────────

// Call once from APP_Initialize.  Registers the ISR with TMR4_CallbackRegister.
// Does NOT start the timer — that happens in INTERPOLATOR_LoadMove.
void INTERPOLATOR_Initialize(void);

// Prepare and launch the execution of one SCurveMove.
// Computes per-axis DDS increments for the entry velocity, sets direction
// GPIOs, resets DDS accumulators, then starts TMR4.
// Must be called from main-loop context (not from ISR).
void INTERPOLATOR_LoadMove(const SCurveMove *move);

// Stop all motion immediately (E-Stop / feed hold).
// Clears step pins, stops TMR4.
void INTERPOLATOR_Stop(void);

// Smoothly decelerate to zero then stop (feed hold with controlled stop).
// Uses the current move's acceleration to compute decel rate.
// ISR continues running and ramps velocity to zero before stopping TMR4.
// MOTION_Tasks must poll g_feed_hold_pending to detect completion via
// STEPPER_FinalizeHold() (same path as hard stop).
void INTERPOLATOR_SoftStop(void);

// Resume after feed hold.  Reloads the same move from where it paused.
// (Simple re-load; full feed-hold position restore is a Phase-2 feature.)
void INTERPOLATOR_Resume(void);

// Returns true while a move is executing.
bool INTERPOLATOR_IsActive(void);

// Returns true once when a move finishes (auto-clears on read).
// Main loop polls this to know when to load the next move.
bool INTERPOLATOR_MoveComplete(void);

// Enable / disable all stepper drivers via the shared EN pin.
void INTERPOLATOR_EnableAll(void);
void INTERPOLATOR_DisableAll(void);

// Feed override scale factor (0.0 = halt, 1.0 = programmed rate, 2.0 = 200%).
// Applied at the next INTERPOLATOR_LoadMove call.
// Range is clamped to [0.1, 2.0] internally.
void INTERPOLATOR_SetFeedOverride(float scale);
float INTERPOLATOR_GetFeedOverride(void);

// Return instant commanded velocity [mm/s] — reads current DDS state.
float INTERPOLATOR_GetInstantVelocity(void);


#ifdef __cplusplus
}
#endif
#endif /* INTERPOLATOR_H */
