/*
 * trajectory.h — S-curve trajectory planner
 *
 * Computes jerk-limited 7-phase velocity profiles and manages a deep lookahead
 * queue of planned moves.  All FPU-heavy math runs in the main loop; the ISR
 * in interpolator.c reads the pre-computed phase tables and evaluates v(t)
 * analytically — no segments, no pre-computed step-interval arrays.
 *
 * 7-phase profile (phases 0-6, boundary times t[0]..t[7]):
 *   Ph 0  J+ : jerk = +J, a ramps 0 → a_max
 *   Ph 1  A  : jerk = 0,  a = a_max  (may be zero-duration)
 *   Ph 2  J- : jerk = -J, a ramps a_max → 0           → arrival at v_nom
 *   Ph 3  C  : cruise at v_nom                         (may be zero-duration)
 *   Ph 4  J- : jerk = -J, a ramps 0 → -a_max
 *   Ph 5  D  : jerk = 0,  a = -a_max                  (may be zero-duration)
 *   Ph 6  J+ : jerk = +J, a ramps -a_max → 0          → arrival at v_exit
 *
 * Degenerate cases (short moves) reduce to 5-phase (no cruise) or 3-phase
 * (no cruise, accel peak < a_max).  Zero-duration phases are harmless: the
 * ISR checks t[i]==t[i+1] and skips instantly.
 */

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "data_structures.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ─── Configuration ────────────────────────────────────────────────────────────

// Deep lookahead queue — 64 moves  (was 16 in GRBL segment model)
// Each SCurveMove is ~144 bytes → 64 × 144 = ~9.2 KB; fine inside 512 KB RAM.
#define TRAJ_QUEUE_SIZE   64u

// ─── SCurveMove ───────────────────────────────────────────────────────────────
//
// One entry in the lookahead queue.  Represents a single G-code linear move
// (or arc chord) as a self-contained 7-phase jerk-limited profile.
//
// ISR evaluation:  for phase ph and local time dt = (t_now - t[ph]):
//   v(t_now) = v[ph] + a[ph]*dt + 0.5*jk[ph]*dt*dt
//   s(t_now) = v[ph]*dt + 0.5*a[ph]*dt*dt + (1/6)*jk[ph]*dt*dt*dt
//
// t[7] is the total move duration; t[0] is always 0.0f.
// Phases with zero duration (t[i]==t[i+1]) are skipped by the ISR.

typedef struct {
    // ── Phase profile (7 phases, 8 boundary points) ──────────────────────────
    float t[8];       // cumulative phase START times [s];  t[0]=0, t[7]=total
    float v[8];       // velocity at start of each phase [mm/s]
    float a[8];       // acceleration at start of each phase [mm/s^2]
    float jk[8];      // jerk (constant) during phase i [mm/s^3]; jk[7] unused

    // ── Geometry ──────────────────────────────────────────────────────────────
    float millimeters;            // total move length [mm]
    float unit_vec[NUM_AXIS];     // normalised direction (sign = forward/back)

    // ── Profile parameters (stored for planner re-use) ────────────────────────
    float nominal_speed;          // planned cruise speed [mm/s]
    float acceleration;           // combined axis-limited acceleration [mm/s^2]
    float J;                      // effective jerk magnitude [mm/s^3]
    float v_entry;                // actual entry speed after blending [mm/s]
    float v_exit;                 // actual exit speed after blending [mm/s]

    // ── Planner junction state ─────────────────────────────────────────────────
    float entry_speed_sqr;        // (mm/s)^2 — set by TRAJECTORY_Recalculate
    float max_entry_speed_sqr;    // junction speed limit squared
    float max_junction_speed_sqr; // centripetal corner limit squared
    bool  speed_locked;           // true → planner must not alter this block
} SCurveMove;


// ─── Public API ───────────────────────────────────────────────────────────────

// Initialise module; call once from APP_Initialize before anything else.
void  TRAJECTORY_Initialize(void);

// Hard reset — flushes queue.  Call on soft reset / E-Stop.
void  TRAJECTORY_Reset(void);

// Add a linear move to the lookahead queue.
// start/end in work coordinates [mm].  feedrate in mm/min.
// entry_v / exit_v are initial guesses; TRAJECTORY_Recalculate will refine them.
// Returns false if queue is full.
bool  TRAJECTORY_AddMove(CoordinatePoint start,
                          CoordinatePoint end,
                          float           feedrate_mm_min,
                          float           entry_v,
                          float           exit_v);

// Junction blending lookahead pass (GRBL-style reverse + forward, jerk-limited).
// Call after every TRAJECTORY_AddMove.
void  TRAJECTORY_Recalculate(void);

// Pop the oldest ready move into *out for the interpolator.
// Returns false if the queue is empty.
bool  TRAJECTORY_GetNextMove(SCurveMove *out);

// Returns number of moves currently in the queue.
uint32_t TRAJECTORY_QueueCount(void);

// Evaluate v(t) [mm/s] at time t [s] from move start.
// Used by the interpolator to compute the DDS increment each tick.
// t is clamped to [0, t[7]].
float TRAJECTORY_VelocityAt(const SCurveMove *move, float t);

// Evaluate distance s(t) [mm] from move start at time t [s].
float TRAJECTORY_PositionAt(const SCurveMove *move, float t);


#ifdef __cplusplus
}
#endif
#endif /* TRAJECTORY_H */
